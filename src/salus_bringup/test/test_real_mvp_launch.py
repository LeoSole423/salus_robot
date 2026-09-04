"""Structural tests for the final real hardware and MVP compositions."""

from pathlib import Path
import subprocess


ROOT = Path(__file__).parents[1]
HARDWARE = ROOT / "launch" / "real_hardware.launch.py"
MVP = ROOT / "launch" / "real_mvp.launch.py"


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
    assert '"imu_expected_frame": "imu_link"' in source
    assert '"gnss_expected_frame": "gps_link"' in source
    assert "ntrip_config_path" in source
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


def test_real_mvp_includes_each_final_block_once() -> None:
    source = MVP.read_text(encoding="utf-8")

    for launch_file in (
        "description_real.launch.py",
        "real_hardware.launch.py",
        "control_real_uart.launch.py",
        "localization_local_real.launch.py",
        "global_localization_real.launch.py",
        "perception_real.launch.py",
        "navigation_real.launch.py",
    ):
        assert source.count(launch_file) == 1
    assert source.count("_include(") == 8  # helper + 7 includes
    for argument in (
        "ntrip_config_path",
        "fcu_url",
        "serial_port",
        "use_keepout",
        "zones_runtime_dir",
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
        "camera",
        "web",
        "systemd",
    ):
        assert forbidden not in lower


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
    for argument in ("ntrip_config_path", "fcu_url", "serial_port"):
        assert argument in result.stdout
