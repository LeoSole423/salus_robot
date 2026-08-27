from salus_web.operator_guard import OperatorLockState
from salus_web.state_projection import (
    project_nav_telemetry,
    project_state,
    unavailable_sensor_info,
)
from salus_interfaces.msg import CapabilityState
from salus_web.ros_gateway import capability_state_label


def test_state_projection_copies_cache_and_adds_lock_aliases() -> None:
    cached = {
        "robot_pose": {"x": 1.0},
        "battery": {"soc": 80.0},
        "capability_profile": "no_obstacle_detection",
        "capabilities": {
            "local_obstacle_detection": {"state": 2, "enabled": False},
        },
    }
    payload = project_state(
        cached,
        OperatorLockState(True, "UI_LOCK_REQUEST", None),
        request_id="req-1",
    )
    cached["robot_pose"]["x"] = 999.0
    assert payload["op"] == "state"
    assert payload["ok"] is True
    assert payload["client_req_id"] == "req-1"
    assert payload["control_locked"] is True
    assert payload["locked"] is True
    assert payload["robot_pose"] == {"x": 1.0}
    assert payload["capability_profile"] == "no_obstacle_detection"
    assert payload["capabilities"]["local_obstacle_detection"]["enabled"] is False


def test_telemetry_and_unavailable_sensor_view_are_explicit() -> None:
    telemetry = project_nav_telemetry(
        {"drive_telemetry": {"speed": 1.2}}, OperatorLockState(False, "", 0.2)
    )
    assert telemetry["op"] == "nav_telemetry"
    assert telemetry["control_locked"] is False
    assert unavailable_sensor_info("lidar") == {
        "op": "sensor_info",
        "tab": "lidar",
        "implemented": False,
    }


def test_capability_state_has_stable_operator_labels() -> None:
    assert capability_state_label(
        CapabilityState.STATE_DISABLED_BY_PROFILE
    ) == "disabled_by_profile"
    assert capability_state_label(CapabilityState.STATE_READY) == "ready"
    assert capability_state_label(255) == "unknown"
