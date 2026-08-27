import math

from sensor_msgs.msg import Imu

from salus_localization.imu_selection_policy import (
    ImuSelectionConfig,
    ImuSelectionState,
    select_imu,
)


def _config(selected_source="imu_primary"):
    return ImuSelectionConfig.create(
        selected_source=selected_source,
        primary_frame="imu_primary_link",
        secondary_frame="imu_secondary_link",
    )


def _message(*, frame="imu_primary_link", stamp=1) -> Imu:
    message = Imu()
    message.header.frame_id = frame
    message.header.stamp.sec = stamp
    message.orientation.w = 1.0
    message.orientation_covariance[0] = 0.1
    return message


def test_only_supported_and_selected_source_is_accepted() -> None:
    config = _config()
    message = _message()
    assert select_imu(ImuSelectionState(), source_id="imu_primary", message=message, config=config).accepted
    assert not select_imu(ImuSelectionState(), source_id="imu_secondary", message=message, config=config).accepted
    assert not select_imu(ImuSelectionState(), source_id="pixhawk", message=message, config=config).accepted


def test_selected_source_requires_its_expected_frame_and_positive_stamp() -> None:
    config = _config()
    assert select_imu(ImuSelectionState(), source_id="imu_primary", message=_message(frame="imu_link"), config=config).reason == "unexpected_frame"
    assert select_imu(ImuSelectionState(), source_id="imu_primary", message=_message(stamp=0), config=config).reason == "invalid_timestamp"


def test_rejects_repeated_or_regressing_timestamps_without_advancing_state() -> None:
    config = _config()
    first = select_imu(ImuSelectionState(), source_id="imu_primary", message=_message(stamp=10), config=config)
    repeated = select_imu(first.state, source_id="imu_primary", message=_message(stamp=10), config=config)
    older = select_imu(first.state, source_id="imu_primary", message=_message(stamp=9), config=config)
    assert first.accepted
    assert repeated.reason == older.reason == "non_monotonic_timestamp"
    assert repeated.state == older.state == first.state


def test_rejects_non_finite_values_and_degenerate_known_quaternion() -> None:
    config = _config()
    non_finite = _message()
    non_finite.angular_velocity.x = math.nan
    assert select_imu(ImuSelectionState(), source_id="imu_primary", message=non_finite, config=config).reason == "non_finite_value"
    degenerate = _message()
    degenerate.orientation.w = 0.0
    assert select_imu(ImuSelectionState(), source_id="imu_primary", message=degenerate, config=config).reason == "degenerate_quaternion"


def test_unknown_orientation_does_not_require_a_quaternion() -> None:
    message = _message()
    message.orientation.w = 0.0
    message.orientation_covariance[0] = -1.0
    assert select_imu(ImuSelectionState(), source_id="imu_primary", message=message, config=_config()).accepted
