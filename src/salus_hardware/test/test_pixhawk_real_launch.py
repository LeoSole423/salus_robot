"""Structural tests for the single-owner physical MAVROS launch."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / "launch/pixhawk_real.launch.py"
REAL_OBSERVATION = ROOT.parent / "salus_bringup/launch/real_observation.launch.py"
RTCM_DELIVERY = ROOT / "launch/pixhawk_rtk_delivery_real.launch.py"


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


def test_pixhawk_rtk_delivery_real_launch_has_exactly_one_adapter() -> None:
    contents = RTCM_DELIVERY.read_text(encoding="utf-8")

    assert contents.count("Node(") == 1
    assert 'package="salus_hardware"' in contents
    assert 'executable="pixhawk_rtk_adapter"' in contents
    assert 'name="pixhawk_rtk_adapter"' in contents
    for forbidden in (
        'package="mavros"',
        'executable="mavros_node"',
        "ntrip_rtcm_source",
        "rs16",
        "uart",
        "serial",
        "localization",
        "heading",
        "nav2",
        "robot_state_publisher",
        "cockpit",
    ):
        assert forbidden not in contents.lower()


def test_pixhawk_rtk_delivery_real_launch_uses_physical_overrides() -> None:
    spec = spec_from_file_location("pixhawk_rtk_delivery_real", RTCM_DELIVERY)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    description = module.generate_launch_description()

    assert len(description.entities) == 1
    contents = RTCM_DELIVERY.read_text(encoding="utf-8")
    assert 'package="salus_hardware"' in contents
    assert 'executable="pixhawk_rtk_adapter"' in contents
    assert 'name="pixhawk_rtk_adapter"' in contents
    for parameter in (
        '"source_status_topic": (',
        '"/salus/hardware/gnss_primary/rtk_source_status"',
        '"rtcm_input_topic": "/salus/hardware/rtcm/corrections"',
        '"gpsraw_topic": "/mavros_node/mavros_node/gps1/raw"',
        '"status_topic": "/salus/hardware/gnss_primary/rtk_status"',
        '"mavros_rtcm_topic": (',
        '"/mavros_node/mavros_node/send_rtcm"',
        '"delivery_backend": "pixhawk_mavros"',
        '"delivery_enabled": True',
        '"stale_timeout_s": 5.0',
        '"status_period_s": 1.0',
        '"use_sim_time": False',
    ):
        assert parameter in contents


def test_pixhawk_rtk_delivery_real_launch_show_args_succeeds() -> None:
    import subprocess

    result = subprocess.run(
        [
            "ros2",
            "launch",
            "salus_hardware",
            "pixhawk_rtk_delivery_real.launch.py",
            "--show-args",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stderr or result.stdout
