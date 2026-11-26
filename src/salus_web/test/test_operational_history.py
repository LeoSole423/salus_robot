from datetime import datetime
from unittest.mock import Mock

import pytest

from salus_web import operational_history as module
from salus_web.operational_history import OperationalHistory
from salus_web.protocol import is_controlled_operation, parse_request, validate_request


def test_patrol_durations_and_observed_stop_are_independent(tmp_path, monkeypatch):
    tick = 100.0
    monkeypatch.setattr(module, "monotonic", lambda: tick)
    logger = Mock()
    history = OperationalHistory(tmp_path / "history.sqlite", "rover", logger)
    source = "patrol_mission_coordinator"
    try:
        history.nav_event("nav", "MANUAL_MODE_CHANGED", 0, {"enabled": True})
        history.nav_event(source, "PATROL_MISSION_STARTED", 0,
                          {"mission_id": "m1", "phase": "DEPART_HOME"})
        tick = 110.0
        history.nav_event(source, "PATROL_LOOP_JOINED", 0, {"mission_id": "m1", "phase": "PATROL"})
        tick = 130.0
        history.observe("moving", True, "controller", {False: ("VEHICLE_STOP_OBSERVED", "stop", 0)})
        history.observe("moving", False, "controller", {False: ("VEHICLE_STOP_OBSERVED", "stop", 0)})
        history.nav_event(source, "PATROL_ROUTE_PAUSED", 1, {
            "mission_id": "m1", "route_status": "ABORTED", "reason_code": "NO_VALID_PATH",
            "private_detail": "discard this",
        })
        tick = 160.0
        history.nav_event(source, "PATROL_MISSION_CANCELLED", 1, {"mission_id": "m1"})
        history.observe("lidar", True, "lidar", {True: ("CAPABILITY_FAILED", "fault", 2)}, initial=True)
    finally:
        history.close()
    patrol, = history.query("history_list", {"view": "patrols"})["items"]
    assert patrol["duration_s"] == 60
    assert patrol["circuit_s"] == 20
    assert patrol["status"] == "cancelled"
    detail = history.query("history_patrol", {"patrol_id": patrol["id"]})
    assert [event["code"] for event in detail["items"]] == [
        "PATROL_MISSION_STARTED", "PATROL_LOOP_JOINED", "VEHICLE_STOP_OBSERVED",
        "PATROL_ROUTE_PAUSED", "PATROL_MISSION_CANCELLED"
    ]
    assert detail["items"][2]["details"] == {}
    assert detail["items"][3]["details"] == {"route_status": "ABORTED", "reason_code": "NO_VALID_PATH"}
    incident, = history.query("history_list", {"category": "fault"})["items"]
    assert incident["patrol_id"] is None
    assert datetime.fromisoformat(incident["timestamp"]).utcoffset().total_seconds() == 0
    logger.assert_not_called()


def test_restart_keeps_checkpointed_time_and_marks_gap(tmp_path, monkeypatch):
    tick = 0.0
    monkeypatch.setattr(module, "monotonic", lambda: tick)
    path = tmp_path / "history.sqlite"
    history = OperationalHistory(path, "rover", Mock())
    history.nav_event("patrol_mission_coordinator", "PATROL_MISSION_STARTED", 0, {"mission_id": "m1"})
    tick = 5.0
    history.checkpoint()
    history.close()

    recovered = OperationalHistory(path, "rover", Mock())
    recovered.close()
    summary, = recovered.query("history_list", {"view": "patrols"})["items"]
    assert summary["status"] == "interrupted"
    assert summary["ended_at"] is None
    assert summary["duration_s"] == 5
    assert summary["incomplete"]
    assert recovered.query("history_list", {"view": "activity", "start": "2099-01-01T00:00:00Z"})["items"] == []


def test_read_pages_have_stable_cursors_and_history_needs_no_lease(tmp_path):
    history = OperationalHistory(tmp_path / "history.sqlite", "test", Mock())
    try:
        history.nav_event("patrol_mission_coordinator", "PATROL_MISSION_STARTED", 0, {"mission_id": "m1"})
        for _ in range(105):
            history.record("controller", "BRAKE_REQUESTED", "stop")
    finally:
        history.close()
    activity = history.query("history_list", {"category": "stop"})
    second = history.query("history_list", {"category": "stop", "before": activity["next"]})
    assert len(activity["items"]) == len(second["items"]) == 50
    assert activity["items"][-1]["sequence"] > second["items"][0]["sequence"]
    patrol_id = activity["items"][0]["patrol_id"]
    first = history.query("history_patrol", {"patrol_id": patrol_id})
    second = history.query("history_patrol", {
        "patrol_id": patrol_id, "through": first["through"], "after": first["next"],
    })
    assert len(first["items"]) == 100
    assert len(second["items"]) == 6
    assert second["next"] is None
    with pytest.raises(ValueError):
        history.query("history_list", {"before": 2**80})
    for op in ("history_list", "history_patrol"):
        assert not is_controlled_operation(validate_request(parse_request({"op": op})))


def test_database_failure_leaves_robot_callbacks_usable(tmp_path):
    path = tmp_path / "directory-instead-of-database"
    path.mkdir()
    logger = Mock()
    history = OperationalHistory(path, "rover", logger)
    history.record("controller", "BRAKE_REQUESTED", "stop")
    history.close()
    logger.assert_called_once()
    with pytest.raises(OSError, match="Historial no disponible"):
        history.query("history_list", {})
