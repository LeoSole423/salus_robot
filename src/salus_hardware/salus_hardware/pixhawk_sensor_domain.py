"""Pure validation rules for sensor samples received through Pixhawk/MAVROS."""

from __future__ import annotations

import math

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


_NANOSECONDS_PER_SECOND = 1_000_000_000
_VALID_COVARIANCE_TYPES = (0, 1, 2, 3)
_VALID_FIX_STATUSES = (-1, 0, 1, 2)


def valid_stamp(message: Imu | NavSatFix) -> bool:
    """Return whether the message has a strictly positive ROS timestamp."""

    seconds = int(message.header.stamp.sec)
    nanoseconds = int(message.header.stamp.nanosec)
    return (
        seconds >= 0
        and 0 <= nanoseconds < _NANOSECONDS_PER_SECOND
        and seconds * _NANOSECONDS_PER_SECOND + nanoseconds > 0
    )


def validate_imu(message: Imu, *, expected_frame: str) -> str:
    """Return ``accepted`` or the first reason a MAVROS IMU sample is rejected."""

    if message.header.frame_id != expected_frame:
        return "unexpected_frame"
    if not valid_stamp(message):
        return "invalid_timestamp"
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
    if not all(math.isfinite(float(value)) for value in values):
        return "non_finite_value"
    if float(message.orientation_covariance[0]) != -1.0:
        quaternion = message.orientation
        norm_squared = sum(
            float(value) ** 2
            for value in (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        )
        if norm_squared <= 1.0e-12:
            return "degenerate_orientation"
    return "accepted"


def validate_gnss(message: NavSatFix, *, expected_frame: str) -> str:
    """Validate structure while preserving an honest ``STATUS_NO_FIX`` sample."""

    if message.header.frame_id != expected_frame:
        return "unexpected_frame"
    if not valid_stamp(message):
        return "invalid_timestamp"
    if int(message.status.status) not in _VALID_FIX_STATUSES:
        return "invalid_fix_status"
    if int(message.position_covariance_type) not in _VALID_COVARIANCE_TYPES:
        return "invalid_covariance_type"
    if not all(math.isfinite(float(value)) for value in message.position_covariance):
        return "non_finite_covariance"
    if int(message.status.status) == NavSatStatus.STATUS_NO_FIX:
        return "accepted"
    if not all(
        math.isfinite(float(value))
        for value in (message.latitude, message.longitude, message.altitude)
    ):
        return "non_finite_position"
    if not -90.0 <= float(message.latitude) <= 90.0:
        return "latitude_out_of_range"
    if not -180.0 <= float(message.longitude) <= 180.0:
        return "longitude_out_of_range"
    return "accepted"
