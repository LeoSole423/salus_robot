"""Pure validation and selection rules for one explicitly chosen IMU source."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional

from sensor_msgs.msg import Imu


PRIMARY_SOURCE_ID = "imu_primary"
SECONDARY_SOURCE_ID = "imu_secondary"
SUPPORTED_SOURCE_IDS = (PRIMARY_SOURCE_ID, SECONDARY_SOURCE_ID)
_NANOSECONDS_PER_SECOND = 1_000_000_000
_MIN_QUATERNION_NORM_SQUARED = 1.0e-12


@dataclass(frozen=True)
class ImuSelectionConfig:
    """The one IMU source authorized to feed the logical IMU topic."""

    selected_source: str
    primary_frame: str
    secondary_frame: str

    @classmethod
    def create(
        cls,
        *,
        selected_source: str,
        primary_frame: str,
        secondary_frame: str,
    ) -> "ImuSelectionConfig":
        selected = str(selected_source)
        if selected not in SUPPORTED_SOURCE_IDS:
            raise ValueError(
                "selected_source must be one of " + ", ".join(SUPPORTED_SOURCE_IDS)
            )
        primary = str(primary_frame)
        secondary = str(secondary_frame)
        if not primary.strip() or not secondary.strip():
            raise ValueError("primary_frame and secondary_frame must not be empty")
        return cls(selected, primary, secondary)

    def expected_frame_for(self, source_id: str) -> str:
        if source_id == PRIMARY_SOURCE_ID:
            return self.primary_frame
        if source_id == SECONDARY_SOURCE_ID:
            return self.secondary_frame
        raise ValueError("unsupported IMU source_id: " + str(source_id))


@dataclass(frozen=True)
class ImuSelectionState:
    """Last accepted source timestamp; no sample means no implicit fallback."""

    last_accepted_stamp_ns: Optional[int] = None


@dataclass(frozen=True)
class ImuSelectionDecision:
    accepted: bool
    reason: str
    state: ImuSelectionState


def message_stamp_ns(message: Imu) -> Optional[int]:
    """Return a strictly positive, well-formed ROS timestamp, if present."""

    seconds = int(message.header.stamp.sec)
    nanoseconds = int(message.header.stamp.nanosec)
    if seconds < 0 or not 0 <= nanoseconds < _NANOSECONDS_PER_SECOND:
        return None
    stamp_ns = seconds * _NANOSECONDS_PER_SECOND + nanoseconds
    return stamp_ns if stamp_ns > 0 else None


def has_only_finite_values(message: Imu) -> bool:
    """Validate every numeric IMU measurement and covariance value."""

    values = (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z,
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z,
        *message.orientation_covariance,
        *message.angular_velocity_covariance,
        *message.linear_acceleration_covariance,
    )
    return all(math.isfinite(float(value)) for value in values)


def has_valid_known_orientation(message: Imu) -> bool:
    """Unknown orientation is allowed; known orientation needs a real quaternion."""

    if float(message.orientation_covariance[0]) == -1.0:
        return True
    quaternion = message.orientation
    norm_squared = (
        float(quaternion.x) ** 2
        + float(quaternion.y) ** 2
        + float(quaternion.z) ** 2
        + float(quaternion.w) ** 2
    )
    return norm_squared > _MIN_QUATERNION_NORM_SQUARED


def select_imu(
    state: ImuSelectionState,
    *,
    source_id: str,
    message: Imu,
    config: ImuSelectionConfig,
) -> ImuSelectionDecision:
    """Accept only the configured source and strictly monotonic valid samples."""

    if source_id not in SUPPORTED_SOURCE_IDS:
        return ImuSelectionDecision(False, "unsupported_source", state)
    if source_id != config.selected_source:
        return ImuSelectionDecision(False, "source_not_selected", state)
    if message.header.frame_id != config.expected_frame_for(source_id):
        return ImuSelectionDecision(False, "unexpected_frame", state)
    stamp_ns = message_stamp_ns(message)
    if stamp_ns is None:
        return ImuSelectionDecision(False, "invalid_timestamp", state)
    if state.last_accepted_stamp_ns is not None and stamp_ns <= state.last_accepted_stamp_ns:
        return ImuSelectionDecision(False, "non_monotonic_timestamp", state)
    if not has_only_finite_values(message):
        return ImuSelectionDecision(False, "non_finite_value", state)
    if not has_valid_known_orientation(message):
        return ImuSelectionDecision(False, "degenerate_quaternion", state)
    return ImuSelectionDecision(True, "accepted", ImuSelectionState(stamp_ns))
