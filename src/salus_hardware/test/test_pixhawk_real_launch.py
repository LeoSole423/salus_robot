"""Structural tests for the single-owner physical MAVROS launch."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / "launch/pixhawk_real.launch.py"
REAL_OBSERVATION = ROOT.parent / "salus_bringup/launch/real_observation.launch.py"


def _launch_module():
    spec = spec_from_file_location("pixhawk_real", LAUNCH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_pixhawk_real_launch_constructs_without_opening_the_fcu() -> None:
    description = _launch_module().generate_launch_description()

    assert len(description.entities) == 2


def test_pixhawk_real_launch_has_exactly_one_sensor_only_mavros_owner() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    assert contents.count("Node(") == 1
    assert 'package="mavros"' in contents
    assert 'executable="mavros_node"' in contents
    assert 'name="mavros_node"' in contents
    assert contents.count("DeclareLaunchArgument(") == 1
    assert '"fcu_url", default_value="/dev/ttyACM0:921600"' in contents
    assert '"gcs_url": ""' in contents
    assert '"tgt_system": 1' in contents
    assert '"tgt_component": 1' in contents
    assert '"fcu_protocol": "v2.0"' in contents


def test_pixhawk_real_launch_preserves_the_legacy_sensor_remaps() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    for source, target in (
        ("mavros_node/data", "/imu/data"),
        ("mavros_node/raw/fix", "/global_position/raw/fix"),
        ("mavros_node/velocity_local", "/local_position/velocity_local"),
        ("mavros_node/odom", "/local_position/odom"),
    ):
        assert source in contents
        assert target in contents


def test_pixhawk_real_launch_excludes_other_hardware_and_control_owners() -> None:
    contents = LAUNCH.read_text(encoding="utf-8").lower()

    for forbidden in (
        "ntrip",
        "rtcm",
        "rslidar",
        "robosense",
        "uart",
        "serial",
        "controller_server",
        "nav2",
        "collision_monitor",
        "nav_command_server",
        "robot_state_publisher",
    ):
        assert forbidden not in contents


def test_real_observation_remains_without_a_mavros_owner() -> None:
    contents = REAL_OBSERVATION.read_text(encoding="utf-8").lower()

    assert 'package="mavros"' not in contents
    assert 'executable="mavros_node"' not in contents
    assert "pixhawk_real.launch.py" not in contents
