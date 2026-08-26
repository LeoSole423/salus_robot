"""Pure validation, pairing and integration policy for wheel odometry."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from .ackermann_odometry import compute_yaw_rate, integrate_planar


STATUS_OK = 1
TRACTION_SOURCE_DRIVE_WHEEL = 2
STEERING_SOURCE_VIRTUAL_CENTER_WHEEL = 4
FIELD_POSITION = 1
FIELD_LINEAR_VELOCITY = 4


@dataclass(frozen=True)
class KinematicSample:
    """One selected kinematic observation, independent of ROS messages."""

    source_id: str
    stamp_s: float
    source_type: int
    status: int
    available_fields: int
    measured_fields: int
    calculated_fields: int
    inferred_fields: int
    value: float


@dataclass(frozen=True)
class KinematicOdometryConfig:
    traction_source_id: str
    steering_source_id: str
    wheelbase_m: float
    max_pair_skew_s: float
    max_dt_s: float

    @classmethod
    def create(
        cls,
        *,
        traction_source_id: str,
        steering_source_id: str,
        wheelbase_m: float,
        max_pair_skew_s: float,
        max_dt_s: float,
    ) -> "KinematicOdometryConfig":
        ids = (str(traction_source_id).strip(), str(steering_source_id).strip())
        values = (wheelbase_m, max_pair_skew_s, max_dt_s)
        if not all(ids):
            raise ValueError("source IDs must not be empty")
        if not all(math.isfinite(float(value)) and float(value) > 0.0 for value in values):
            raise ValueError("wheelbase_m, max_pair_skew_s and max_dt_s must be finite and positive")
        return cls(ids[0], ids[1], *(float(value) for value in values))


@dataclass(frozen=True)
class KinematicOdometryState:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0
    baseline_stamp_s: Optional[float] = None
    pending_traction: Optional[KinematicSample] = None
    pending_steering: Optional[KinematicSample] = None


@dataclass(frozen=True)
class OdometryEmission:
    stamp_s: float
    x_m: float
    y_m: float
    yaw_rad: float
    speed_mps: float
    yaw_rate_rps: float


@dataclass(frozen=True)
class KinematicOdometryUpdate:
    state: KinematicOdometryState
    emission: Optional[OdometryEmission] = None


def accept_traction(
    state: KinematicOdometryState, sample: KinematicSample, config: KinematicOdometryConfig
) -> KinematicOdometryUpdate:
    """Accept one traction sample, clearing only its pending value when invalid."""
    if sample.source_id != config.traction_source_id:
        return KinematicOdometryUpdate(state)
    if not _consumable(sample, TRACTION_SOURCE_DRIVE_WHEEL, FIELD_LINEAR_VELOCITY):
        return KinematicOdometryUpdate(_replace(state, pending_traction=None))
    return _pair_if_ready(_replace(state, pending_traction=sample), config)


def accept_steering(
    state: KinematicOdometryState, sample: KinematicSample, config: KinematicOdometryConfig
) -> KinematicOdometryUpdate:
    """Accept one steering sample, clearing only its pending value when invalid."""
    if sample.source_id != config.steering_source_id:
        return KinematicOdometryUpdate(state)
    if not _consumable(sample, STEERING_SOURCE_VIRTUAL_CENTER_WHEEL, FIELD_POSITION):
        return KinematicOdometryUpdate(_replace(state, pending_steering=None))
    return _pair_if_ready(_replace(state, pending_steering=sample), config)


def _consumable(sample: KinematicSample, expected_source: int, required_field: int) -> bool:
    masks = (sample.measured_fields, sample.calculated_fields, sample.inferred_fields)
    if sample.status != STATUS_OK or sample.source_type != expected_source:
        return False
    if (
        not math.isfinite(sample.stamp_s)
        or sample.stamp_s <= 0.0
        or not math.isfinite(sample.value)
        or not (sample.available_fields & required_field)
    ):
        return False
    if any(mask & ~sample.available_fields for mask in masks):
        return False
    if (masks[0] & masks[1]) or (masks[0] & masks[2]) or (masks[1] & masks[2]):
        return False
    return (masks[0] | masks[1] | masks[2]) == sample.available_fields


def _pair_if_ready(state: KinematicOdometryState, config: KinematicOdometryConfig) -> KinematicOdometryUpdate:
    traction, steering = state.pending_traction, state.pending_steering
    if traction is None or steering is None:
        return KinematicOdometryUpdate(state)
    consumed = _replace(state, pending_traction=None, pending_steering=None)
    if abs(traction.stamp_s - steering.stamp_s) > config.max_pair_skew_s:
        return KinematicOdometryUpdate(consumed)
    stamp_s = max(traction.stamp_s, steering.stamp_s)
    speed_mps, steer_rad = traction.value, steering.value
    yaw_rate_rps = compute_yaw_rate(speed_mps, steer_rad, config.wheelbase_m)
    if consumed.baseline_stamp_s is not None and stamp_s <= consumed.baseline_stamp_s:
        # Never publish a non-monotonic odometry timestamp. A real regression
        # also invalidates the old time base; an exact duplicate is ignored.
        baseline = None if stamp_s < consumed.baseline_stamp_s else consumed.baseline_stamp_s
        return KinematicOdometryUpdate(_replace(consumed, baseline_stamp_s=baseline))
    reset_baseline = (
        consumed.baseline_stamp_s is None
        or stamp_s - consumed.baseline_stamp_s > config.max_dt_s
    )
    if reset_baseline:
        next_state = _replace(consumed, baseline_stamp_s=stamp_s)
    else:
        x_m, y_m, yaw_rad = integrate_planar(
            consumed.x_m, consumed.y_m, consumed.yaw_rad,
            speed_mps, yaw_rate_rps, stamp_s - consumed.baseline_stamp_s,
        )
        next_state = _replace(consumed, x_m=x_m, y_m=y_m, yaw_rad=yaw_rad, baseline_stamp_s=stamp_s)
    return KinematicOdometryUpdate(
        next_state,
        OdometryEmission(stamp_s, next_state.x_m, next_state.y_m, next_state.yaw_rad, speed_mps, yaw_rate_rps),
    )


def _replace(state: KinematicOdometryState, **changes) -> KinematicOdometryState:
    values = {
        "x_m": state.x_m, "y_m": state.y_m, "yaw_rad": state.yaw_rad,
        "baseline_stamp_s": state.baseline_stamp_s,
        "pending_traction": state.pending_traction, "pending_steering": state.pending_steering,
    }
    values.update(changes)
    return KinematicOdometryState(**values)
