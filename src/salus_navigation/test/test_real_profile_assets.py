"""Structural contracts for the software-only real navigation composition."""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).parents[1]
CONFIG = PACKAGE / "config"
LAUNCH = PACKAGE / "launch"


def _yaml(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _use_sim_time_values(document):
    if isinstance(document, dict):
        for key, value in document.items():
            if key == "use_sim_time":
                yield value
            else:
                yield from _use_sim_time_values(value)
    elif isinstance(document, list):
        for value in document:
            yield from _use_sim_time_values(value)


def _diff_paths(left, right, prefix=()):
    if isinstance(left, dict) and isinstance(right, dict):
        for key in sorted(set(left) | set(right), key=str):
            yield from _diff_paths(left.get(key), right.get(key), prefix + (str(key),))
    elif isinstance(left, list) and isinstance(right, list):
        assert len(left) == len(right), prefix
        for index, (left_item, right_item) in enumerate(zip(left, right)):
            yield from _diff_paths(left_item, right_item, prefix + (str(index),))
    elif left != right:
        yield prefix, left, right


def test_real_yaml_is_the_parsed_sim_profile_plus_only_use_sim_time():
    sim = _yaml(CONFIG / "nav2_core_sim.yaml")
    real = _yaml(CONFIG / "nav2_core_real.yaml")
    differences = list(_diff_paths(sim, real))
    assert differences
    assert all(path[-1] == "use_sim_time" for path, _, _ in differences)
    assert all(before is True and after is False for _, before, after in differences)


def test_real_yaml_has_no_sim_time_true_and_keeps_single_clean_scan_source():
    real = _yaml(CONFIG / "nav2_core_real.yaml")
    assert all(value is False for value in _use_sim_time_values(real))
    text = (CONFIG / "nav2_core_real.yaml").read_text(encoding="utf-8")
    assert "use_sim_time: true" not in text.lower()
    assert text.count("topic: /scan_clean") == 2
    assert "observation_sources: scan" in text


def test_real_core_has_exactly_the_frozen_processes_and_lifecycle_targets():
    source = (LAUNCH / "navigation_core_real.launch.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    node_names = []
    for call in [node for node in ast.walk(tree) if isinstance(node, ast.Call)]:
        if isinstance(call.func, ast.Name) and call.func.id == "Node":
            for keyword in call.keywords:
                if keyword.arg == "name" and isinstance(keyword.value, ast.Constant):
                    node_names.append(keyword.value.value)
    assert node_names == [
        "nav_observer", "path_health", "navigation_profile_coordinator",
        "lifecycle_manager_navigation", "nav2_startup_coordinator",
    ]
    assert '("nav2_planner", "planner_server", "planner_server")' in source
    assert '("nav2_controller", "controller_server", "controller_server")' in source
    assert '("nav2_bt_navigator", "bt_navigator", "bt_navigator")' in source
    assert '("nav2_behaviors", "behavior_server", "behavior_server")' in source
    assert '"autostart": False' in source
    assert '"node_names": [name for _, _, name in nodes]' in source
    assert '"require_clock_progress": False' in source
    assert '"obstacle_detection_required": True' in source


def test_real_top_level_composes_zones_collision_core_and_one_command_server():
    source = (LAUNCH / "navigation_real.launch.py").read_text(encoding="utf-8")
    assert source.count("IncludeLaunchDescription(") == 3
    assert source.count('executable="nav_command_server"') == 1
    assert '"zones_runtime_dir", default_value="runtime/zones"' in source
    assert '"use_keepout", default_value="true"' in source
    for value in (
        '"cmd_vel_safe_topic": "/cmd_vel_safe"',
        '"cmd_vel_final_topic": "/cmd_vel_final"',
        '"safety_scan_topic": "/scan_clean"',
        '"gps_topic": "/salus/gps/fix"',
        '"fromll_service": "/fromLL"',
        '"obstacle_detection_required": True',
    ):
        assert value in source


def test_real_launches_are_software_only_and_have_no_second_authority():
    sources = "\n".join(
        (LAUNCH / name).read_text(encoding="utf-8")
        for name in (
            "navigation_zones_real.launch.py",
            "navigation_core_real.launch.py",
            "navigation_real.launch.py",
        )
    ).lower()
    assert "gazebo" not in sources
    assert "mavros" not in sources
    assert "uart" not in sources
    assert "rs16" not in sources
    assert "hardware" not in sources
    assert "salus_control" not in sources
    assert sources.count('executable="nav_command_server"') == 1


def test_real_zones_launch_has_one_manager_and_fixed_wall_clock():
    source = (LAUNCH / "navigation_zones_real.launch.py").read_text(encoding="utf-8")
    assert source.count("Node(") == 1
    assert 'executable="zones_manager"' in source
    assert '"use_sim_time": False' in source
    assert 'DeclareLaunchArgument("use_keepout", default_value="true")' in source
    assert 'DeclareLaunchArgument("runtime_dir", default_value="runtime/zones")' in source
