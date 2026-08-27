from pathlib import Path

import pytest
from sensor_msgs.msg import Imu

from salus_localization.orientation_source_selector import (
    COURSE_OVER_GROUND,
    EXTERNAL_HEADING,
    OrientationSelectionPolicy,
    normalize_orientation_source,
)


def _orientation(stamp: int = 1, frame: str = "base_footprint") -> Imu:
    message = Imu()
    message.header.stamp.sec = stamp
    message.header.frame_id = frame
    message.orientation.w = 1.0
    message.orientation_covariance[0] = 0.1
    message.orientation_covariance[4] = 0.1
    message.orientation_covariance[8] = 0.1
    return message


def test_source_selection_is_explicit_and_rejects_fallback_names() -> None:
    assert normalize_orientation_source(" COURSE_OVER_GROUND ") == COURSE_OVER_GROUND
    assert normalize_orientation_source("external_heading") == EXTERNAL_HEADING
    for invalid in ("", "auto", "fallback", "imu", None):
        with pytest.raises(ValueError):
            normalize_orientation_source(invalid)


def test_policy_accepts_only_selected_source_and_monotonic_valid_samples() -> None:
    policy = OrientationSelectionPolicy(COURSE_OVER_GROUND, "base_footprint")
    assert not policy.evaluate(EXTERNAL_HEADING, _orientation()).accepted
    assert policy.evaluate(COURSE_OVER_GROUND, _orientation()).accepted
    repeated = policy.evaluate(COURSE_OVER_GROUND, _orientation())
    assert not repeated.accepted and repeated.reason == "non_monotonic_timestamp"
    assert policy.evaluate(COURSE_OVER_GROUND, _orientation(2)).accepted


@pytest.mark.parametrize(
    ("mutate", "reason"),
    [
        (lambda message: setattr(message.header, "frame_id", "imu_link"), "unexpected_frame"),
        (lambda message: setattr(message.header.stamp, "sec", 0), "non_positive_timestamp"),
        (lambda message: setattr(message.orientation, "w", 0.0), "degenerate_orientation"),
        (lambda message: setattr(message.orientation, "w", 2.0), "unnormalized_orientation"),
        (lambda message: message.orientation_covariance.__setitem__(8, 0.0), "orientation_unavailable"),
    ],
)
def test_policy_rejects_malformed_orientation(mutate, reason: str) -> None:
    policy = OrientationSelectionPolicy(EXTERNAL_HEADING, "base_footprint")
    message = _orientation()
    mutate(message)
    decision = policy.evaluate(EXTERNAL_HEADING, message)
    assert not decision.accepted and decision.reason == reason


def test_console_entry_point_is_packaged() -> None:
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "orientation_source_selector = " in setup
