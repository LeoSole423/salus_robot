from types import SimpleNamespace

from salus_navigation.patrol_mission_coordinator import (
    patrol_spec_from_request,
    resolved_patrol_spec,
)
from salus_navigation.route_model import RouteWaypoint


DEFAULTS = (2.0, 20.0, 5)


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
    assert spec.leg_spacing_m == 2.0
    assert spec.chunk_span_m == 20.0
    assert spec.chunk_max_waypoints == 5
    assert spec.loop.waypoints[0].yaw_deg != spec.loop.waypoints[0].yaw_deg


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
