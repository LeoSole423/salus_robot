"""Characterize explicit capability profiles without ROS timing."""

import pytest

from salus_hardware.capability_profile import (
    PROFILE_NO_OBSTACLE_DETECTION,
    PROFILE_OBSTACLE_DETECTION,
    declarations_for_profile,
    normalize_profile,
)


def test_profiles_are_explicit_and_normalized() -> None:
    assert normalize_profile("OBSTACLE_DETECTION") == PROFILE_OBSTACLE_DETECTION
    assert normalize_profile(" no_obstacle_detection ") == PROFILE_NO_OBSTACLE_DETECTION
    for invalid in ("", "auto", "fallback", "no_lidar", None):
        with pytest.raises(ValueError):
            normalize_profile(invalid)


def test_no_obstacle_profile_never_claims_detection_ready() -> None:
    capabilities = declarations_for_profile(
        PROFILE_NO_OBSTACLE_DETECTION, ready_state=7, disabled_state=2,
    )
    assert {item.capability_id for item in capabilities} == {
        "local_obstacle_detection", "lidar_primary",
    }
    assert all(item.state == 2 for item in capabilities)
    assert all(not item.enabled and not item.required for item in capabilities)


def test_normal_profile_requires_enabled_detection() -> None:
    capabilities = declarations_for_profile(
        PROFILE_OBSTACLE_DETECTION, ready_state=7, disabled_state=2,
    )
    assert all(item.state == 7 for item in capabilities)
    assert all(item.enabled and item.required for item in capabilities)
