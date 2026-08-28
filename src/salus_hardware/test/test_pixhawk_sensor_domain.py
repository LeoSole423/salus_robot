import math

from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from salus_hardware.pixhawk_sensor_domain import validate_gnss, validate_imu


def _imu() -> Imu:
    message = Imu()
    message.header.stamp.sec = 1
    message.header.frame_id = "base_link"
    message.orientation.w = 1.0
    return message


def _fix(status=NavSatStatus.STATUS_FIX) -> NavSatFix:
    message = NavSatFix()
    message.header.stamp.sec = 1
    message.header.frame_id = "base_link"
    message.status.status = status
    message.latitude = -31.0
    message.longitude = -64.0
    message.altitude = 400.0
    return message


def test_imu_preserves_live_base_frame_and_rejects_bad_samples() -> None:
    assert validate_imu(_imu(), expected_frame="base_link") == "accepted"
    message = _imu()
    message.header.frame_id = "invented_mount"
    assert validate_imu(message, expected_frame="base_link") == "unexpected_frame"
    message = _imu()
    message.angular_velocity.x = math.nan
    assert validate_imu(message, expected_frame="base_link") == "non_finite_value"


def test_gnss_preserves_no_fix_even_with_unknown_position() -> None:
    message = _fix(NavSatStatus.STATUS_NO_FIX)
    message.latitude = math.nan
    message.longitude = math.nan
    message.altitude = math.nan
    assert validate_gnss(message, expected_frame="base_link") == "accepted"


def test_gnss_rejects_invalid_fixed_position_and_covariance() -> None:
    message = _fix()
    message.latitude = 91.0
    assert validate_gnss(message, expected_frame="base_link") == "latitude_out_of_range"
    message = _fix()
    message.position_covariance[0] = math.nan
    assert validate_gnss(message, expected_frame="base_link") == "non_finite_covariance"
