"""Pure compatibility translation from ``CmdVelFinal`` semantics."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .control_logic import (
    COMMAND_SOURCE_AUTO,
    COMMAND_SOURCE_MANUAL,
    COMMAND_SOURCE_SAFETY,
    COMMAND_SOURCE_UNKNOWN,
    command_from_cmd_vel,
)


VALID_COMMAND_SOURCES = frozenset(
    {
        COMMAND_SOURCE_UNKNOWN,
        COMMAND_SOURCE_AUTO,
        COMMAND_SOURCE_MANUAL,
        COMMAND_SOURCE_SAFETY,
    }
)


@dataclass(frozen=True, slots=True)
class LegacyVehicleCommandConfig:
    max_speed_mps: float = 4.0
    max_reverse_mps: float = 1.3
    vx_deadband_mps: float = 0.1
    vx_min_effective_mps: float = 0.75
    wheelbase_m: float = 0.94
    steering_limit_rad: float = 0.5235987756
    operational_steering_limit_rad: float = 0.3141592654
    manual_operational_steering_limit_rad: float = 0.5235987756
    drive_enabled: bool = True
    valid_for_s: float = 0.7

    def __post_init__(self) -> None:
        finite = (
            self.max_speed_mps,
            self.max_reverse_mps,
            self.vx_deadband_mps,
            self.vx_min_effective_mps,
            self.wheelbase_m,
            self.steering_limit_rad,
            self.operational_steering_limit_rad,
            self.manual_operational_steering_limit_rad,
            self.valid_for_s,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError("vehicle command configuration must be finite")
        if self.max_speed_mps < 0.0 or self.max_reverse_mps < 0.0:
            raise ValueError("speed limits must be nonnegative")
        if self.vx_deadband_mps < 0.0 or self.vx_min_effective_mps < 0.0:
            raise ValueError("speed thresholds must be nonnegative")
        if self.wheelbase_m <= 0.0 or self.steering_limit_rad <= 0.0:
            raise ValueError("wheelbase and steering limit must be positive")
        if self.operational_steering_limit_rad <= 0.0:
            raise ValueError("operational steering limit must be positive")
        if self.manual_operational_steering_limit_rad <= 0.0:
            raise ValueError("manual steering limit must be positive")
        if self.valid_for_s <= 0.0:
            raise ValueError("valid_for_s must be positive")


@dataclass(frozen=True, slots=True)
class VehicleCommandValue:
    source: int
    drive_enabled: bool
    emergency_stop: bool
    brake_ratio: float
    speed_mps: float
    steering_angle_rad: float
    valid_for_s: float
    valid_input: bool = True
    reason: str = "legacy_compatibility"


def translate_legacy_command(
    *,
    linear_x_mps: float,
    angular_z_rps: float,
    brake_pct: int,
    source: int,
    config: LegacyVehicleCommandConfig,
) -> VehicleCommandValue:
    """Translate legacy fields while preserving its brake-as-E-stop behavior."""
    if not math.isfinite(linear_x_mps) or not math.isfinite(angular_z_rps):
        return _safe_invalid(config, "nonfinite_legacy_command")
    if int(source) not in VALID_COMMAND_SOURCES:
        return _safe_invalid(config, "invalid_legacy_source")

    desired = command_from_cmd_vel(
        linear_x=linear_x_mps,
        angular_z=angular_z_rps,
        brake_pct=brake_pct,
        max_speed_mps=config.max_speed_mps,
        max_reverse_mps=config.max_reverse_mps,
        vx_deadband_mps=config.vx_deadband_mps,
        vx_min_effective_mps=config.vx_min_effective_mps,
        wheelbase_m=config.wheelbase_m,
        steering_limit_rad=config.steering_limit_rad,
        invert_steer=False,
        auto_drive_enabled=config.drive_enabled,
        reverse_brake_pct=0,
        operational_steering_limit_rad=config.operational_steering_limit_rad,
        manual_operational_steering_limit_rad=(
            config.manual_operational_steering_limit_rad
        ),
        command_source=source,
    )
    return VehicleCommandValue(
        source=int(source),
        drive_enabled=bool(desired.drive_enabled),
        emergency_stop=bool(desired.estop),
        brake_ratio=float(desired.brake_pct) / 100.0,
        speed_mps=float(desired.speed_mps),
        steering_angle_rad=(
            0.0 if desired.estop else float(desired.applied_steer_rad)
        ),
        valid_for_s=config.valid_for_s,
    )


def _safe_invalid(
    config: LegacyVehicleCommandConfig, reason: str
) -> VehicleCommandValue:
    return VehicleCommandValue(
        source=COMMAND_SOURCE_SAFETY,
        drive_enabled=False,
        emergency_stop=True,
        brake_ratio=1.0,
        speed_mps=0.0,
        steering_angle_rad=0.0,
        valid_for_s=config.valid_for_s,
        valid_input=False,
        reason=reason,
    )
