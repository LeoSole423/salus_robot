"""Immutable inputs and outputs for navigation evaluation."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple


class ExpectedTurn(str, Enum):
    """Expected initial direction in the vehicle frame."""

    LEFT = "left"
    RIGHT = "right"
    STRAIGHT = "straight"
    ANY = "any"


@dataclass(frozen=True)
class Pose2D:
    """Planar pose in metres and radians."""

    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class GoalSpec:
    """Goal expressed relative to the scenario spawn pose."""

    goal_id: str
    forward_m: float
    lateral_m: float
    yaw_offset_rad: float
    timeout_s: float
    expected_turn: ExpectedTurn
    reverse_allowed: bool = False


@dataclass(frozen=True)
class ScenarioSpec:
    """Versioned, deterministic scenario definition."""

    scenario_id: str
    world: str
    spawn: Pose2D
    goals: Tuple[GoalSpec, ...]
    schema_version: int = 1


@dataclass(frozen=True)
class TimedPose:
    """Observed pose and twist at a monotonic timestamp."""

    stamp_s: float
    pose: Pose2D
    linear_x_mps: float = 0.0
    angular_z_rps: float = 0.0


@dataclass(frozen=True)
class TimedCommand:
    """Velocity command at a named control stage."""

    stamp_s: float
    linear_x_mps: float
    angular_z_rps: float
    stage: str = "cmd_vel"


@dataclass(frozen=True)
class TimedFinalCommand:
    """Final velocity command with its public authority metadata."""

    stamp_s: float
    linear_x_mps: float
    angular_z_rps: float
    brake_pct: int
    source: int


@dataclass(frozen=True)
class TimedVehicleCommand:
    """Ackermann command observed at the vehicle boundary."""

    stamp_s: float
    source: int
    drive_enabled: bool
    emergency_stop: bool
    brake_ratio: float
    speed_mps: float
    steering_angle_rad: float


@dataclass(frozen=True)
class TimedDriveTelemetry:
    """Measured drive state, normalized to SI radians and metres per second."""

    stamp_s: float
    ready: bool
    fresh: bool
    drive_enabled: bool
    estop: bool
    speed_valid: bool
    steer_valid: bool
    control_source: str
    speed_mps_measured: float
    steer_rad_measured: float
    brake_applied_pct: int


@dataclass(frozen=True)
class TimedControllerStatus:
    """Effective controller command parsed from its observer-only JSON status."""

    stamp_s: float
    source: str
    fresh: bool
    drive_enabled: bool
    estop: bool
    speed_mps: float
    brake_pct: int
    requested_linear_x_mps: float
    requested_angular_z_rps: float
    requested_steer_rad: float
    applied_steer_rad: float
    steering_limit_used_rad: float
    steer_saturated: bool
    speed_limited: bool
    min_speed_enforced: bool


@dataclass(frozen=True)
class TimedControllerTelemetry:
    """Requested automatic command and configured Ackermann limits from JSON."""

    stamp_s: float
    requested_speed_mps: float
    requested_steer_rad: float
    applied_steer_rad: float
    steering_limit_deg: float
    operational_steering_limit_deg: float
    effective_steering_limit_deg: float


@dataclass(frozen=True)
class TrackingMetrics:
    """Path tracking measurements for one goal."""

    sample_count: int
    cross_track_rms_m: float
    cross_track_p95_m: float
    cross_track_max_m: float
    heading_p95_rad: float
    traveled_m: float
    path_efficiency: float


@dataclass(frozen=True)
class SignMetrics:
    """Causal steering-command versus yaw-response measurements."""

    eligible_count: int
    mismatch_count: int
    mismatch_fraction: Optional[float]
    first_command_sign: int
    first_response_sign: int


@dataclass(frozen=True)
class ArrivalMetrics:
    """Arrival, overshoot and settling measurements."""

    first_entry_s: Optional[float]
    exits_after_entry: int
    minimum_distance_m: float
    final_distance_m: float
    post_success_distance_m: Optional[float]
    overshoot_m: float


@dataclass(frozen=True)
class LocalizationMetrics:
    """Estimate error against simulation ground truth."""

    sample_count: int
    position_rmse_m: float
    position_p95_m: float
    yaw_rmse_rad: float
    final_position_error_m: float
