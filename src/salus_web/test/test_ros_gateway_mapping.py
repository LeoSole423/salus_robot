import json
import math
from pathlib import Path

from salus_interfaces.srv import CameraPtzState
from salus_web.protocol import parse_request, validate_request
from salus_web.ros_gateway import build_ros_request


def _request(payload):
    return validate_request(parse_request(payload))


def test_gateway_does_not_shadow_rclpy_node_client_storage() -> None:
    source = (Path(__file__).parents[1] / "salus_web" / "ros_gateway.py").read_text(
        encoding="utf-8"
    )
    assert "self._service_clients =" in source
    assert "self._clients =" not in source


def test_goal_mapping_preserves_arrays_and_auto_yaw() -> None:
    request = build_ros_request(_request({
        "op": "set_goal_ll",
        "waypoints": [{"lat": -31.0, "lon": -64.0}],
        "loop": False,
    }))
    assert list(request.lats) == [-31.0]
    assert list(request.lons) == [-64.0]
    assert math.isnan(request.yaws_deg[0])
    assert request.lat == -31.0
    assert request.loop is False


def test_route_mapping_keeps_roles_actions_and_options() -> None:
    request = build_ros_request(_request({
        "op": "set_route_ll",
        "waypoints": [
            {
                "lat": -31.0,
                "lon": -64.0,
                "yaw_deg": 20.0,
                "role": "home",
                "actions": [{"type": "brake_hold", "duration_s": 1.0}],
            }
        ],
        "loop": True,
        "leg_spacing_m": 3.0,
        "chunk_span_m": 30.0,
        "chunk_max_waypoints": 12,
    }))
    assert list(request.waypoint_roles) == ["home"]
    assert json.loads(request.waypoint_action_jsons[0])[0]["type"] == "brake_hold"
    assert request.loop is True
    assert request.leg_spacing_m == 3.0
    assert request.chunk_max_waypoints == 12


def test_patrol_mapping_matches_cockpit_nested_shape() -> None:
    request = build_ros_request(_request({
        "op": "set_patrol_ll",
        "patrol_mission": {
            "loop_waypoints": [
                {"lat": -31.0, "lon": -64.0},
                {"lat": -31.1, "lon": -64.1},
            ],
            "home_waypoint": {"lat": -31.2, "lon": -64.2, "yaw_deg": 90.0},
            "return_waypoints": [],
            "depart_waypoints": [{"lat": -31.15, "lon": -64.15}],
            "depart_entry_loop_index": 1,
        },
    }))
    assert list(request.loop_lats) == [-31.0, -31.1]
    assert request.home_yaw_deg == 90.0
    assert request.depart_entry_loop_index == 1
    assert list(request.depart_lons) == [-64.15]


def test_camera_mapping_preserves_optional_axis_contract() -> None:
    request = build_ros_request(_request({
        "op": "camera_ptz_move", "relative": True, "pan_deg": 15.0,
    }))
    assert request.relative is True
    assert request.apply_pan is True
    assert request.apply_tilt is False
    assert request.apply_zoom is False

    save = build_ros_request(_request({
        "op": "camera_ptz_set_preset", "preset": "home", "save_zoom": True,
    }))
    assert save.preset == "home"
    assert save.save_zoom is True

    state = build_ros_request(_request({"op": "get_camera_ptz_state"}))
    assert isinstance(state, CameraPtzState.Request)
