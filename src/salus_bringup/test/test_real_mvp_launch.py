"""Structural tests for the final real hardware and MVP compositions."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import subprocess

from launch.actions import IncludeLaunchDescription


ROOT = Path(__file__).parents[1]
PACKAGE_XML = ROOT / "package.xml"
HARDWARE = ROOT / "launch" / "real_hardware.launch.py"
MVP = ROOT / "launch" / "real_mvp.launch.py"


def _real_hardware_module():
    spec = spec_from_file_location("real_hardware", HARDWARE)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _launch_configuration_name(value) -> str:
    return "".join(token.text for token in value.variable_name)


def test_real_hardware_has_exactly_the_final_physical_owners() -> None:
    source = HARDWARE.read_text(encoding="utf-8")

    for launch_file in (
        "pixhawk_real.launch.py",
        "pixhawk_sensor_inputs.launch.py",
        "ntrip_rtcm_source_real.launch.py",
        "pixhawk_rtk_delivery_real.launch.py",
        "rs16_real.launch.py",
    ):
        assert source.count(launch_file) == 1
    assert source.count("_include(") == 6  # helper + 5 includes
    assert 'default_value="/dev/ttyACM0:921600"' in source
    assert '"imu_expected_frame": "base_link"' in source
    assert '"gnss_expected_frame": "base_link"' in source
    assert "ntrip_config_path" in source
    assert "rs16_config_path" in source
    assert "active_source_id" in source

    lower = source.lower()
    for forbidden in (
        "controller",
        "uart",
        "serial",
        "nav2",
        "localization",
        "perception",
        "collision_monitor",
        "shadow",
        "sim",
        "gazebo",
        "camera",
        "web",
    ):
        assert forbidden not in lower


def test_real_hardware_keeps_ntrip_and_rs16_configs_independent() -> None:
    description = _real_hardware_module().generate_launch_description()
    includes = [
        entity
        for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]

    assert len(includes) == 5
    ntrip_arguments = dict(includes[2].launch_arguments)
    rs16_arguments = dict(includes[4].launch_arguments)

    assert _launch_configuration_name(ntrip_arguments["config_path"]) == (
        "ntrip_config_path"
    )
    assert _launch_configuration_name(rs16_arguments["config_path"]) == (
        "rs16_config_path"
    )

    sensor_arguments = dict(includes[1].launch_arguments)
    assert sensor_arguments["imu_expected_frame"] == "base_link"
    assert sensor_arguments["gnss_expected_frame"] == "base_link"


def test_real_mvp_includes_each_final_block_once() -> None:
    source = MVP.read_text(encoding="utf-8")

    for launch_file in (
        "description_real.launch.py",
        "real_hardware.launch.py",
        "control_real_uart.launch.py",
        "localization_local_real.launch.py",
        "global_localization_real.launch.py",
        "perception_real.launch.py",
        "camera_real.launch.py",
        "navigation_real.launch.py",
        "web_bridge.launch.py",
    ):
        assert source.count(launch_file) == 1
    assert source.count("_include(") == 10  # helper + 9 includes
    for argument in (
        "ntrip_config_path",
        "fcu_url",
        "serial_port",
        "use_keepout",
        "zones_runtime_dir",
        "patrol_runtime_dir",
        "patrol_battery_guard_topic",
        "patrol_battery_state_topic",
        "web_gps_fix_topic",
        "web_heading_odometry_topic",
        "camera_host",
        "camera_port",
        "camera_channel",
        "camera_presets_file",
    ):
        assert argument in source

    lower = source.lower()
    for forbidden in (
        "real_observation",
        "shadow",
        "coexistence",
        "dry-run",
        "legacy",
        "gazebo",
        "sim",
        "systemd",
    ):
        assert forbidden not in lower

    assert '"salus_web",\n            "web_bridge.launch.py",' in source
    assert '"web_gps_fix_topic", default_value="/salus/gps/fix"' in source
    assert '"gps_fix_topic": web_gps_fix_topic' in source
    assert '"web_heading_odometry_topic",\n            default_value="/odometry/global"' in source
    assert '"heading_odometry_topic": web_heading_odometry_topic' in source
    assert '"salus_hardware",\n            "camera_real.launch.py",' in source
    assert source.count('"camera_real.launch.py"') == 1
    assert "use_sim_time" not in source
    assert "enable_control_lock" not in source
    assert "control_lock_start_locked" not in source


def test_real_mvp_runtime_dependency_is_declared() -> None:
    package_xml = PACKAGE_XML.read_text(encoding="utf-8")
    assert "<exec_depend>salus_description</exec_depend>" in package_xml


def test_real_hardware_show_args_does_not_start_devices() -> None:
    result = subprocess.run(
        [
            "ros2",
            "launch",
            "salus_bringup",
            "real_hardware.launch.py",
            "--show-args",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stderr or result.stdout
    assert "ntrip_config_path" in result.stdout
    assert "fcu_url" in result.stdout


def test_real_mvp_show_args_does_not_start_devices() -> None:
    result = subprocess.run(
        [
            "ros2",
            "launch",
            "salus_bringup",
            "real_mvp.launch.py",
            "--show-args",
        ],
        capture_output=True,
        text=True,
        timeout=30,
        check=False,
    )
    assert result.returncode == 0, result.stderr or result.stdout
    for argument in (
        "ntrip_config_path", "fcu_url", "serial_port", "patrol_runtime_dir",
    ):
        assert argument in result.stdout
