from math import nan
from threading import RLock
from types import SimpleNamespace
from unittest.mock import Mock

from salus_navigation.patrol_mission_coordinator import (
    PatrolMissionCoordinator,
    patrol_spec_from_request,
    resolved_patrol_spec,
    route_state_belongs_to_dispatch,
)
from salus_navigation.route_model import RouteWaypoint


DEFAULTS = (35.0, 120.0, 5)


def request(**overrides):
    values = {
        "loop_lats": [-31.0, -31.0001, -31.0001],
        "loop_lons": [-64.0, -64.0, -64.0001],
        "loop_yaws_deg": [],
        "loop_waypoint_action_jsons": ["", "", ""],
        "home_lat": -31.0,
        "home_lon": -64.0,
        "home_yaw_deg": 0.0,
        "return_lats": [], "return_lons": [], "return_yaws_deg": [],
        "return_waypoint_action_jsons": [],
        "depart_lats": [], "depart_lons": [], "depart_yaws_deg": [],
        "depart_waypoint_action_jsons": [],
        "depart_entry_loop_index": 0,
        "leg_spacing_m": 0.0, "chunk_span_m": 0.0, "chunk_max_waypoints": 0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def mapped(route, x_offset=0.0):
    return [RouteWaypoint(**{**point.__dict__, "map_x": x_offset + index,
                             "map_y": float(index)})
            for index, point in enumerate(route.waypoints)]


def test_request_keeps_legacy_empty_yaws_and_applies_declared_defaults():
    spec, error = patrol_spec_from_request(request(), DEFAULTS)
    assert error == ""
    assert spec.leg_spacing_m == 35.0
    assert spec.chunk_span_m == 120.0
    assert spec.chunk_max_waypoints == 5
    assert spec.loop.waypoints[0].yaw_deg != spec.loop.waypoints[0].yaw_deg


def test_request_preserves_the_distinction_between_automatic_and_explicit_yaws():
    spec, error = patrol_spec_from_request(
        request(loop_yaws_deg=[nan, 90.0, nan]), DEFAULTS)

    assert error == ""
    assert [point.yaw_explicit for point in spec.loop.waypoints] == [False, True, False]


def test_request_rejects_mismatched_actions_before_replacing_a_mission():
    spec, error = patrol_spec_from_request(
        request(loop_waypoint_action_jsons=["{}"]), DEFAULTS)
    assert spec is None
    assert "actions" in error


def test_resolved_document_resolves_yaw_and_preserves_action_alignment():
    spec, error = patrol_spec_from_request(
        request(
            loop_waypoint_action_jsons=[
                "", '[{"type":"brake_hold","duration_s":1.0,"brake_pct":30}]', ""]), DEFAULTS, )
    assert error == ""
    result = resolved_patrol_spec(spec, {
        "home": [RouteWaypoint(**{**spec.home.__dict__, "map_x": 0.0, "map_y": 0.0})],
        "loop": mapped(spec.loop), "depart": [], "return": [],
    })
    assert all(point.map_x is not None for point in result.loop.waypoints)
    assert all(point.yaw_deg == point.yaw_deg for point in result.loop.waypoints)
    assert result.loop.actions[1].startswith('[{"type":"brake_hold"')


def test_replacement_does_not_adopt_cancelled_route_mission_id():
    assert not route_state_belongs_to_dispatch("", "old-route", "old-route")
    assert route_state_belongs_to_dispatch("", "old-route", "new-route")
    assert route_state_belongs_to_dispatch("new-route", "old-route", "new-route")
    assert not route_state_belongs_to_dispatch("new-route", "old-route", "third-route")

def test_cancelled_patrol_cannot_be_started_by_a_late_replacement_callback():
    coordinator = SimpleNamespace(
        _lock=RLock(), _preparation_id=7, _preparation_status="cancelled",
        _start_spec=Mock(), _reject=Mock())
    future = Mock()

    PatrolMissionCoordinator._finish_replacement(coordinator, future, object(), 7)

    future.result.assert_not_called()
    coordinator._start_spec.assert_not_called()
    coordinator._reject.assert_not_called()


def test_replacement_does_not_adopt_route_from_the_previous_patrol():
    machine = SimpleNamespace(state=SimpleNamespace(active=True, mission_id="new"), pause=Mock())
    coordinator = SimpleNamespace(
        _lock=RLock(), _machine=machine, _route_mission_id="",
        _superseded_route_mission_id="", _delegated_preparation_id=8)
    future = Mock()
    future.result.return_value = SimpleNamespace(
        ok=True, preparation_id=8, preparation_rejection="",
        parent_patrol_id="old", mission_id="old-route", status="PAUSED")

    PatrolMissionCoordinator._on_route_state(coordinator, future)

    machine.pause.assert_not_called()
    assert coordinator._route_mission_id == ""
