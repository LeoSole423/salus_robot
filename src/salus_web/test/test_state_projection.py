from salus_web.operator_guard import OperatorLockState
from salus_web.state_projection import (
    project_nav_telemetry,
    project_state,
    unavailable_sensor_info,
)


def test_state_projection_copies_cache_and_adds_lock_aliases() -> None:
    cached = {"robot_pose": {"x": 1.0}, "battery": {"soc": 80.0}}
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
