"""Structural contract tests for the real global localization profile."""

from __future__ import annotations

import ast
from pathlib import Path
import shutil
import subprocess

import pytest
import yaml

from salus_localization.datum_profile import (
    DEFAULT_DATUM_LAT,
    DEFAULT_DATUM_LON,
    DEFAULT_DATUM_YAW_DEG,
    resolve_selected_datum,
)


PACKAGE = Path(__file__).parents[1]
LAUNCH = PACKAGE / "launch" / "global_localization_real.launch.py"
CONFIG = PACKAGE / "config" / "localization_global_real.yaml"
DATUMS = PACKAGE / "config" / "datums.yaml"


def _executable_literals() -> list[str]:
    tree = ast.parse(LAUNCH.read_text(encoding="utf-8"))
    values = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != "Node":
            continue
        executable = next(
            keyword.value
            for keyword in node.keywords
            if keyword.arg == "executable"
        )
        assert isinstance(executable, ast.Constant)
        values.append(executable.value)
    return values


def _config() -> dict:
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def test_real_launch_starts_exactly_the_five_frozen_processes() -> None:
    assert _executable_literals() == [
        "global_stationary_gates",
        "gps_course_heading",
        "orientation_source_selector",
        "navsat_transform_node",
        "ekf_node",
    ]
    assert LAUNCH.read_text(encoding="utf-8").count("Node(") == 5


def test_real_launch_has_no_forbidden_runtime_owner_or_backend() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    lines = source.splitlines()
    if tree.body and isinstance(tree.body[0], ast.Expr):
        if isinstance(tree.body[0].value, ast.Constant) and isinstance(
            tree.body[0].value.value, str
        ):
            for index in range(tree.body[0].lineno - 1, tree.body[0].end_lineno):
                lines[index] = ""
    code = "\n".join(
        "" if line.lstrip().startswith("#") else line for line in lines
    ).lower()
    for forbidden in (
        "localization_local",
        "ackermann_odometry",
        "mavros",
        "ntrip",
        "rtcm",
        "compass",
        "external_heading",
        "yaw_artificial",
        "robot_state_publisher",
        "static_transform_publisher",
        "nav2",
        "uart",
        "rslidar",
        "perception",
        "sim_gps_normalizer",
        "sim_external_heading_from_odom",
        "map_gps_absolute_measurement",
        "gazebo",
    ):
        assert forbidden not in code


def test_real_launch_pins_topics_and_typed_course_heading_gate() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    expected = (
        '"gps_topic": "/salus/gps/fix"',
        '"odom_topic": "/odometry/local"',
        '"drive_telemetry_topic": "/controller/drive_telemetry"',
        '"rtk_status_topic": "/salus/hardware/gnss_primary/rtk_status"',
        '"rtk_status_wire_type": "gnss_rtk_status"',
        '"output_topic": "/gps/course_heading"',
        '"base_frame": "base_footprint"',
        '"require_rtk": True',
        '"rtk_status_max_age_s": 2.5',
    )
    assert all(fragment in source for fragment in expected)
    assert '"selected_source": "course_over_ground"' in source
    assert '"course_topic": "/gps/course_heading"' in source
    assert '"output_topic": "/localization/orientation"' in source
    assert '"expected_frame": "base_footprint"' in source


def test_real_launch_has_the_required_navsat_remappings() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    for remap in (
        '("gps/fix", "/salus/gps/fix")',
        '("odometry/filtered", "/odometry/global")',
        '("odometry/gps", "/odometry/gps")',
        '("imu", "/localization/orientation")',
    ):
        assert remap in source
    assert '"odometry/filtered", "/odometry/local"' not in source


def test_real_launch_configures_stationary_gates_without_a_second_tf_owner() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    for fragment in (
        '"odom_topic": "/odometry/local"',
        '"imu_topic": "/salus/imu/data"',
        '"drive_telemetry_topic": "/controller/drive_telemetry"',
        '"stationary_speed_threshold_mps": 0.03',
        '"drive_telemetry_timeout_s": 0.5',
    ):
        assert fragment in source
    assert '"publish_tf": False' not in source
    assert "odom -> base_footprint" not in source


def test_ros2_launch_show_args_is_available_for_the_real_profile() -> None:
    ros2 = shutil.which("ros2")
    if ros2 is None:
        pytest.skip("ROS 2 is only available in the project container")
    result = subprocess.run(
        [ros2, "launch", "salus_localization", "global_localization_real.launch.py", "--show-args"],
        check=False,
        capture_output=True,
        text=True,
        timeout=20,
    )
    assert result.returncode == 0, result.stderr
    assert "datum_lat" in result.stdout
    assert "datum_lon" in result.stdout
    assert "datum_yaw_deg" in result.stdout


def test_global_ekf_is_the_only_configured_map_to_odom_authority() -> None:
    parameters = _config()["salus_global_ekf"]["ros__parameters"]
    assert parameters["frequency"] == 30.0
    assert parameters["sensor_timeout"] == 0.2
    assert parameters["two_d_mode"] is True
    assert parameters["publish_tf"] is True
    assert parameters["map_frame"] == "map"
    assert parameters["odom_frame"] == "odom"
    assert parameters["base_link_frame"] == "base_footprint"
    assert parameters["world_frame"] == "map"
    assert parameters["odom0"] == "/odometry/local_global"
    assert parameters["odom0_config"] == [
        False, False, False, False, False, False,
        True, True, False, False, False, True,
        False, False, False,
    ]
    assert parameters["odom1"] == "/odometry/gps"
    assert parameters["odom1_config"] == [
        True, True, False, False, False, False,
        False, False, False, False, False, False,
        False, False, False,
    ]
    assert parameters["odom1_differential"] is False
    assert parameters["odom1_relative"] is False
    assert parameters["imu0"] == "/imu/data_global"
    assert parameters["imu0_config"][11] is True
    assert sum(parameters["imu0_config"]) == 1
    assert parameters["imu1"] == "/localization/orientation"
    assert parameters["imu1_config"][5] is True
    assert sum(parameters["imu1_config"]) == 1
    assert parameters["imu1_differential"] is False
    assert parameters["imu1_relative"] is False
    assert "odom2" not in parameters


def test_datum_matches_legacy_selected_and_fallback_contract() -> None:
    document = yaml.safe_load(DATUMS.read_text(encoding="utf-8"))
    assert document["selected_id"] == "gps-16-6-2026-16-17-09"
    selected = next(
        item for item in document["datums"] if item["id"] == document["selected_id"]
    )
    assert (
        selected["lat"],
        selected["lon"],
        selected["yaw_deg"],
    ) == (-31.4859026607927, -64.24097358249034, 0.0)
    assert resolve_selected_datum(str(PACKAGE))[:3] == (
        -31.4859026607927,
        -64.24097358249034,
        0.0,
    )

    assert (DEFAULT_DATUM_LAT, DEFAULT_DATUM_LON, DEFAULT_DATUM_YAW_DEG) == (
        -31.4858037,
        -64.2410570,
        0.0,
    )


def test_invalid_datum_document_uses_operational_fallback(tmp_path) -> None:
    config_dir = tmp_path / "config"
    config_dir.mkdir()
    (config_dir / "datums.yaml").write_text(
        "version: 1\nselected_id: broken\ndatums:\n- id: broken\n  lat: nan\n  lon: 0\n  yaw_deg: 0\n",
        encoding="utf-8",
    )
    lat, lon, yaw, path = resolve_selected_datum(str(tmp_path))
    assert (lat, lon, yaw) == (
        DEFAULT_DATUM_LAT,
        DEFAULT_DATUM_LON,
        DEFAULT_DATUM_YAW_DEG,
    )
    assert path == str(config_dir / "datums.yaml")


@pytest.mark.parametrize("argument", ["datum_lat", "datum_lon", "datum_yaw_deg"])
def test_real_launch_exposes_explicit_datum_overrides(argument: str) -> None:
    assert f'DeclareLaunchArgument("{argument}"' in LAUNCH.read_text(
        encoding="utf-8"
    )
