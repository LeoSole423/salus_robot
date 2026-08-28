import math

import pytest
from sensor_msgs.msg import NavSatFix, NavSatStatus

from salus_localization.gnss_selection_policy import GnssSelectionPolicy


def _fix(stamp=1, status=NavSatStatus.STATUS_FIX) -> NavSatFix:
    message = NavSatFix()
    message.header.stamp.sec = stamp
    message.header.frame_id = "base_link"
    message.status.status = status
    message.latitude = -31.0
    message.longitude = -64.0
    message.altitude = 400.0
    return message


def test_configuration_requires_explicit_supported_source() -> None:
    with pytest.raises(ValueError):
        GnssSelectionPolicy("auto", "base_link")


def test_only_selected_source_and_expected_frame_are_accepted() -> None:
    policy = GnssSelectionPolicy("gnss_primary", "base_link")
    assert not policy.evaluate("gnss_secondary", _fix()).accepted
    message = _fix()
    message.header.frame_id = "other"
    assert policy.evaluate("gnss_primary", message).reason == "unexpected_frame"
    assert policy.evaluate("gnss_primary", _fix()).accepted


def test_timestamps_must_be_strictly_monotonic() -> None:
    policy = GnssSelectionPolicy("gnss_primary", "base_link")
    assert policy.evaluate("gnss_primary", _fix(2)).accepted
    assert policy.evaluate("gnss_primary", _fix(2)).reason == "non_monotonic_timestamp"
    assert policy.evaluate("gnss_primary", _fix(1)).reason == "non_monotonic_timestamp"


def test_no_fix_with_unknown_position_is_forwarded_honestly() -> None:
    message = _fix(status=NavSatStatus.STATUS_NO_FIX)
    message.latitude = math.nan
    message.longitude = math.nan
    message.altitude = math.nan
    assert GnssSelectionPolicy("gnss_primary", "base_link").evaluate(
        "gnss_primary", message
    ).accepted
