"""Structural checks for the isolated real Collision Monitor profile."""

from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
CONFIG = PACKAGE / "config" / "collision_monitor_real.yaml"
LAUNCH = PACKAGE / "launch" / "collision_monitor_real.launch.py"


def _parameters() -> dict:
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return document["collision_monitor"]["ros__parameters"]


def test_real_profile_preserves_the_frozen_boundary_parameters() -> None:
    parameters = _parameters()

    assert parameters["use_sim_time"] is False
    assert {
        key: parameters[key]
        for key in (
            "base_frame_id",
            "odom_frame_id",
            "cmd_vel_in_topic",
            "cmd_vel_out_topic",
            "state_topic",
            "expected_planner_frequency",
            "transform_tolerance",
            "state_update_rate",
            "source_timeout",
            "stop_pub_timeout",
        )
    } == {
        "base_frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "cmd_vel_in_topic": "/cmd_vel",
        "cmd_vel_out_topic": "/cmd_vel_safe",
        "state_topic": "/collision_monitor_state",
        "expected_planner_frequency": 10.0,
        "transform_tolerance": 0.5,
        "state_update_rate": 10.0,
        "source_timeout": 1.0,
        "stop_pub_timeout": 0.5,
    }


def test_real_profile_preserves_all_four_frozen_polygons() -> None:
    parameters = _parameters()
    assert parameters["polygons"] == [
        "footprint",
        "stop_zone",
        "critical_slow_zone",
        "slow_zone",
    ]

    expected = {
        "footprint": {
            "action_type": "stop",
            "min_points": 3,
            "points": [1.05, 0.38, 1.05, -0.38, -0.12, -0.38, -0.12, 0.38],
        },
        "stop_zone": {
            "action_type": "approach",
            "min_points": 3,
            "points": [2.05, 0.68, 2.05, -0.68, -0.30, -0.68, -0.30, 0.68],
            "time_before_collision": 2.0,
            "simulation_time_step": 0.1,
            "polygon_pub_topic": "/stop_zone_raw",
        },
        "critical_slow_zone": {
            "action_type": "slowdown",
            "min_points": 3,
            "points": [3.50, 0.98, 3.50, -0.98, -0.30, -0.98, -0.30, 0.98],
            "slowdown_ratio": 0.4375,
            "polygon_pub_topic": "/critical_slow_zone_raw",
        },
        "slow_zone": {
            "action_type": "slowdown",
            "min_points": 3,
            "points": [5.35, 1.18, 5.35, -1.18, -0.30, -1.18, -0.30, 1.18],
            "slowdown_ratio": 0.75,
            "polygon_pub_topic": "/slow_zone_raw",
        },
    }
    for name, values in expected.items():
        assert {key: parameters[name][key] for key in values} == values


def test_real_launch_has_only_collision_monitor_and_its_lifecycle_manager() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    assert contents.count("Node(") == 2
    assert contents.count('package="nav2_collision_monitor"') == 1
    assert contents.count('package="nav2_lifecycle_manager"') == 1
    assert 'executable="collision_monitor"' in contents
    assert 'executable="lifecycle_manager"' in contents
    assert 'name="collision_monitor"' in contents
    assert 'name="lifecycle_manager_collision_monitor_real"' in contents
    assert '"autostart": True' in contents
    assert '"node_names": ["collision_monitor"]' in contents
    assert '"use_sim_time": False' in contents

    forbidden = (
        "nav_command_server",
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "waypoint_follower",
        "smoother_server",
        "keepout",
        "zones_manager",
        "route_executor",
        "patrol",
        "localization",
        "robot_state_publisher",
        "rslidar",
        "robosense",
        "mavros",
        "ntrip",
        "uart",
        "safety_command_passthrough",
    )
    lower_contents = contents.lower()
    assert all(item not in lower_contents for item in forbidden)


def test_real_config_uses_only_the_clean_scan_observation_source() -> None:
    parameters = _parameters()
    assert parameters["observation_sources"] == ["scan"]
    assert parameters["scan"] == {
        "type": "scan",
        "topic": "/scan_clean",
        "enabled": True,
    }
