"""Pure validation and watchdog policy for canonical vehicle commands."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .control_logic import (
    COMMAND_SOURCE_AUTO,
    COMMAND_SOURCE_MANUAL,
    COMMAND_SOURCE_SAFETY,
    COMMAND_SOURCE_UNKNOWN,
)


VALID_SOURCES = frozenset(
    {
        COMMAND_SOURCE_UNKNOWN,
        COMMAND_SOURCE_AUTO,
        COMMAND_SOURCE_MANUAL,
        COMMAND_SOURCE_SAFETY,
    }
)


@dataclass(frozen=True, slots=True)
class CanonicalCommandConfig:
    max_forward_speed_mps: float = 4.0
    max_reverse_speed_mps: float = 1.3
    max_steering_angle_rad: float = 0.5235987756
    max_valid_for_s: float = 0.7
    max_future_skew_s: float = 0.1

    def __post_init__(self) -> None:
        values = (
            self.max_forward_speed_mps,
            self.max_reverse_speed_mps,
            self.max_steering_angle_rad,
            self.max_valid_for_s,
            self.max_future_skew_s,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("canonical consumer limits must be finite and nonnegative")
        if self.max_steering_angle_rad == 0.0 or self.max_valid_for_s == 0.0:
            raise ValueError("steering and validity limits must be positive")


@dataclass(frozen=True, slots=True)
class CanonicalCommandSample:
    stamp_ns: int
    source: int
    drive_enabled: bool
    emergency_stop: bool
    brake_ratio: float
    speed_mps: float
    steering_angle_rad: float
    steering_angle_velocity_rad_s: float
    acceleration_mps2: float
    jerk_mps3: float
    valid_for_s: float


@dataclass(frozen=True, slots=True)
class EffectiveCanonicalCommand:
    source: int
    drive_enabled: bool
    emergency_stop: bool
    brake_ratio: float
    speed_mps: float
    steering_angle_rad: float
    valid: bool
    reason: str


def safe_effective_command(reason: str) -> EffectiveCanonicalCommand:
    return EffectiveCanonicalCommand(
        source=COMMAND_SOURCE_SAFETY,
        drive_enabled=False,
        emergency_stop=True,
        brake_ratio=1.0,
        speed_mps=0.0,
        steering_angle_rad=0.0,
        valid=False,
        reason=reason,
    )


class CanonicalCommandConsumer:
    """Stateful pure consumer with ROS-stamp validation and monotonic expiry."""

    def __init__(self, config: CanonicalCommandConfig) -> None:
        self.config = config
        self.last_stamp_ns: int | None = None
        self.received_at_s: float | None = None
        self.effective = safe_effective_command("no_command")
        self._validity_s = 0.0

    def ingest(
        self,
        sample: CanonicalCommandSample,
        *,
        ros_now_ns: int,
        monotonic_now_s: float,
    ) -> EffectiveCanonicalCommand:
        reason = self._invalid_reason(sample, ros_now_ns)
        if reason:
            self.received_at_s = None
            self.effective = safe_effective_command(reason)
            return self.effective

        self.last_stamp_ns = sample.stamp_ns
        self.received_at_s = monotonic_now_s
        self._validity_s = min(sample.valid_for_s, self.config.max_valid_for_s)
        inhibited = sample.emergency_stop or not sample.drive_enabled
        if sample.emergency_stop:
            reason = "emergency_stop"
        elif not sample.drive_enabled:
            reason = "drive_disabled"
        elif sample.brake_ratio > 0.0:
            reason = "service_brake"
        else:
            reason = "accepted"
        self.effective = EffectiveCanonicalCommand(
            source=sample.source,
            drive_enabled=sample.drive_enabled and not sample.emergency_stop,
            emergency_stop=sample.emergency_stop,
            brake_ratio=1.0 if sample.emergency_stop else sample.brake_ratio,
            speed_mps=0.0 if inhibited or sample.brake_ratio > 0.0 else sample.speed_mps,
            steering_angle_rad=0.0 if inhibited else sample.steering_angle_rad,
            valid=True,
            reason=reason,
        )
        return self.effective

    def tick(self, monotonic_now_s: float) -> EffectiveCanonicalCommand:
        if self.received_at_s is None:
            return self.effective
        if monotonic_now_s - self.received_at_s > self._validity_s:
            self.received_at_s = None
            self.effective = safe_effective_command("watchdog_timeout")
        return self.effective

    def _invalid_reason(
        self, sample: CanonicalCommandSample, ros_now_ns: int
    ) -> str:
        finite = (
            sample.brake_ratio,
            sample.speed_mps,
            sample.steering_angle_rad,
            sample.steering_angle_velocity_rad_s,
            sample.acceleration_mps2,
            sample.jerk_mps3,
            sample.valid_for_s,
        )
        if not all(math.isfinite(value) for value in finite):
            return "nonfinite_command"
        if sample.source not in VALID_SOURCES:
            return "invalid_source"
        if sample.stamp_ns <= 0:
            return "invalid_stamp"
        if self.last_stamp_ns is not None and sample.stamp_ns <= self.last_stamp_ns:
            return "nonmonotonic_stamp"
        future_ns = sample.stamp_ns - ros_now_ns
        if future_ns > round(self.config.max_future_skew_s * 1_000_000_000):
            return "future_stamp"
        if sample.valid_for_s <= 0.0:
            return "invalid_validity"
        validity_s = min(sample.valid_for_s, self.config.max_valid_for_s)
        if ros_now_ns - sample.stamp_ns > round(validity_s * 1_000_000_000):
            return "stale_on_arrival"
        if not 0.0 <= sample.brake_ratio <= 1.0:
            return "invalid_brake_ratio"
        if not (
            -self.config.max_reverse_speed_mps
            <= sample.speed_mps
            <= self.config.max_forward_speed_mps
        ):
            return "speed_out_of_range"
        if abs(sample.steering_angle_rad) > self.config.max_steering_angle_rad:
            return "steering_out_of_range"
        return ""
