"""Structural safety tests for the read-only real coexistence profile."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "real_observation.launch.py"


def _launch_module():
    spec = spec_from_file_location("real_observation", LAUNCH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_real_observation_launch_is_constructible_without_hardware() -> None:
    """Construction performs no device, network, or ROS graph I/O."""
    description = _launch_module().generate_launch_description()
    assert len(description.entities) == 14


def test_real_observation_has_only_the_approved_observers() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    for executable in (
        "pixhawk_sensor_inputs.launch.py",
        "rtk_gnss_observation.launch.py",
        'executable="legacy_drive_measurement_node"',
        'executable="legacy_vehicle_command_node"',
        'executable="vehicle_command_comparison_node"',
    ):
        assert executable in contents

    assert contents.count("Node(") == 3
    assert contents.count("_include(") == 3


def test_real_observation_hard_codes_rtcm_delivery_to_dry_run() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    assert '"delivery_backend": "disabled"' in contents
    assert '"delivery_enabled": "false"' in contents
    assert 'DeclareLaunchArgument("delivery_backend"' not in contents
    assert 'DeclareLaunchArgument("delivery_enabled"' not in contents
    assert '"legacy_rtcm_type": "uint8_multi_array"' in contents


def test_real_observation_keeps_command_translation_shadow_only() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    assert 'default_value="/cmd_vel_final"' in contents
    assert '"input_topic": legacy_command_topic' in contents
    assert '"legacy_topic": legacy_command_topic' in contents
    assert '"output_topic": "/vehicle/command_shadow"' in contents
    assert '"shadow_topic": "/vehicle/command_shadow"' in contents
    assert '"diagnostics_topic": "/vehicle/command_shadow/diagnostics"' in contents
    assert '"/cmd_vel_final"' not in contents.split("parameters=[", 1)[1]


def test_real_observation_excludes_every_actuating_or_global_authority() -> None:
    contents = LAUNCH.read_text(encoding="utf-8").lower()

    for forbidden in (
        'executable="controller_server_node"',
        'package="mavros"',
        'executable="mavros_node"',
        "ntrip",
        "serial",
        "uart",
        "rslidar",
        "robosense",
        "robot_state_publisher",
        'executable="ekf_node"',
        "navsat_transform",
        'package="nav2_',
        "collision_monitor",
        'executable="nav_command_server"',
        "route_executor",
        "patrol_mission",
        'package="salus_web"',
        "camera",
        "mediamtx",
        '"/tf"',
        '"/mavros_node/send_rtcm"',
    ):
        assert forbidden not in contents
