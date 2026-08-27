"""Characterize explicit capability profiles without ROS timing."""

import pytest

from salus_hardware.capability_profile import (
    PROFILE_NO_OBSTACLE_DETECTION,
    PROFILE_OBSTACLE_DETECTION,
    declarations_for_profile,
    normalize_profile,
    normalize_selection,
    observed_state,
    VALID_IMU_SOURCES,
    VALID_ORIENTATION_SOURCES,
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
    by_id = {item.capability_id: item for item in capabilities}
    assert set(by_id) == {
        "local_obstacle_detection",
        "lidar_primary",
        "local_motion_imu",
        "global_orientation",
    }
    obstacle_items = (
        by_id["local_obstacle_detection"], by_id["lidar_primary"]
    )
    assert all(item.state == 2 for item in obstacle_items)
    assert all(not item.enabled and not item.required for item in obstacle_items)
    assert by_id["local_motion_imu"].enabled
    assert by_id["global_orientation"].enabled


def test_normal_profile_requires_enabled_detection() -> None:
    capabilities = declarations_for_profile(
        PROFILE_OBSTACLE_DETECTION, ready_state=7, disabled_state=2,
    )
    assert all(item.state == 7 for item in capabilities)
    assert all(item.enabled and item.required for item in capabilities)


def test_sensor_selections_are_explicit_and_reported_as_sources() -> None:
    assert normalize_selection(
        " IMU_SECONDARY ", field="imu_source", choices=VALID_IMU_SOURCES
    ) == "imu_secondary"
    with pytest.raises(ValueError):
        normalize_selection("auto", field="imu_source", choices=VALID_IMU_SOURCES)
    with pytest.raises(ValueError):
        normalize_selection(
            "fallback", field="orientation_source", choices=VALID_ORIENTATION_SOURCES
        )
    capabilities = declarations_for_profile(
        PROFILE_OBSTACLE_DETECTION,
        ready_state=7,
        disabled_state=2,
        imu_source="imu_secondary",
        orientation_source="external_heading",
    )
    by_id = {item.capability_id: item for item in capabilities}
    assert by_id["local_motion_imu"].source_ids == ("imu_secondary",)
    assert by_id["global_orientation"].source_ids == ("external_heading",)
    assert "no automatic fallback" in by_id["global_orientation"].detail


def test_observed_state_reports_unavailable_ready_and_stale_without_fallback() -> None:
    states = {"unavailable_state": 3, "stale_state": 5, "ready_state": 8}
    assert observed_state(
        now_s=10.0, last_sample_s=None, timeout_s=0.5, **states
    ) == 3
    assert observed_state(
        now_s=10.0, last_sample_s=9.6, timeout_s=0.5, **states
    ) == 8
    assert observed_state(
        now_s=10.2, last_sample_s=9.6, timeout_s=0.5, **states
    ) == 5
    with pytest.raises(ValueError):
        observed_state(now_s=10.0, last_sample_s=None, timeout_s=0.0, **states)
