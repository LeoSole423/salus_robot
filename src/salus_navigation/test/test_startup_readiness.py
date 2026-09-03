"""Characterize the pure Nav2 startup gate without ROS timing."""

from dataclasses import replace
from pathlib import Path

from salus_navigation.startup_readiness import (
    ReadinessSnapshot, StartupPolicy, StartupState, missing_requirements,
)
from diagnostic_msgs.msg import DiagnosticStatus


def ready_snapshot(**changes) -> ReadinessSnapshot:
    """Return a fully ready fixture with selected overrides."""
    snapshot = ReadinessSnapshot(
        clock_progressive=True,
        odometry_progressive=True,
        odometry_finite=True,
        transform_available=True,
        transform_fresh=True,
        scan_valid=True,
        scan_fresh=True,
        keepout_required=True,
        keepout_ready=True,
        lifecycle_manager_ready=True,
    )
    return replace(snapshot, **changes)


def test_each_missing_input_blocks_startup_with_explicit_reason() -> None:
    cases = {
        "clock_progressive": "CLOCK_NOT_PROGRESSIVE",
        "odometry_progressive": "ODOMETRY_NOT_PROGRESSIVE",
        "odometry_finite": "ODOMETRY_INVALID",
        "transform_available": "TF_UNAVAILABLE",
        "transform_fresh": "TF_STALE",
        "scan_valid": "SCAN_INVALID",
        "scan_fresh": "SCAN_STALE",
        "keepout_ready": "KEEPOUT_NOT_READY",
        "lifecycle_manager_ready": "LIFECYCLE_MANAGER_UNAVAILABLE",
    }
    for field, reason in cases.items():
        snapshot = ready_snapshot(**{field: False})
        assert reason in missing_requirements(snapshot)
        policy = StartupPolicy()
        assert not policy.observe(snapshot)
        assert policy.state is StartupState.WAITING_INPUTS
        assert policy.reason == reason


def test_simulation_without_progressive_clock_still_blocks() -> None:
    snapshot = ready_snapshot(clock_required=True, clock_progressive=False)
    assert "CLOCK_NOT_PROGRESSIVE" in missing_requirements(snapshot)
    assert not StartupPolicy().observe(snapshot)


def test_real_profile_does_not_require_clock_progress() -> None:
    snapshot = ready_snapshot(clock_required=False, clock_progressive=False)
    assert "CLOCK_NOT_PROGRESSIVE" not in missing_requirements(snapshot)
    assert StartupPolicy().observe(snapshot)


def test_real_profile_keeps_each_non_clock_gate_independent() -> None:
    for field, reason in {
        "odometry_progressive": "ODOMETRY_NOT_PROGRESSIVE",
        "transform_available": "TF_UNAVAILABLE",
        "transform_fresh": "TF_STALE",
        "scan_valid": "SCAN_INVALID",
        "scan_fresh": "SCAN_STALE",
        "keepout_ready": "KEEPOUT_NOT_READY",
        "lifecycle_manager_ready": "LIFECYCLE_MANAGER_UNAVAILABLE",
    }.items():
        snapshot = ready_snapshot(clock_required=False, **{field: False})
        assert reason in missing_requirements(snapshot)
        assert not StartupPolicy().observe(snapshot)


def test_coordinator_switches_clock_subscription_and_tf_reference_by_profile() -> None:
    source = (
        Path(__file__).parents[1]
        / "salus_navigation"
        / "nav2_startup_coordinator.py"
    ).read_text(encoding="utf-8")
    assert 'declare_parameter("require_clock_progress", True)' in source
    assert 'if self._require_clock_progress:' in source
    assert 'Clock, "/clock", self._on_clock' in source
    assert "self.get_clock().now().nanoseconds" in source
    assert "time.monotonic()" not in source[
        source.index("    def _update_tf"):source.index("    def _snapshot")
    ]


def test_keepout_is_optional_only_when_explicitly_disabled() -> None:
    snapshot = ready_snapshot(keepout_required=False, keepout_ready=False)
    assert "KEEPOUT_NOT_READY" not in missing_requirements(snapshot)
    assert StartupPolicy().observe(snapshot)


def test_scan_is_optional_only_in_explicit_no_detection_profile() -> None:
    snapshot = ready_snapshot(
        obstacle_detection_required=False, scan_valid=False, scan_fresh=False,
    )
    assert "SCAN_INVALID" not in missing_requirements(snapshot)
    assert "SCAN_STALE" not in missing_requirements(snapshot)
    assert StartupPolicy().observe(snapshot)


def test_ready_inputs_request_start_exactly_once_and_can_activate() -> None:
    policy = StartupPolicy()
    assert policy.observe(ready_snapshot())
    assert policy.state is StartupState.STARTING
    assert not policy.observe(ready_snapshot())
    policy.activation_succeeded()
    assert policy.state is StartupState.ACTIVE
    assert policy.reason == "ALL_NAV2_NODES_ACTIVE"


def test_rejected_activation_is_terminal() -> None:
    policy = StartupPolicy()
    assert policy.observe(ready_snapshot())
    policy.activation_failed("LIFECYCLE_START_REJECTED")
    assert policy.state is StartupState.FAILED
    assert not policy.observe(ready_snapshot())


def test_diagnostic_status_constants_keep_humble_byte_type() -> None:
    assert isinstance(DiagnosticStatus.OK, bytes)
    assert len(DiagnosticStatus.WARN) == 1
