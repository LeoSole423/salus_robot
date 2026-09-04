from pathlib import Path


def test_pixhawk_sensor_launch_is_read_only_and_namespaced() -> None:
    launch = (
        Path(__file__).parents[1] / "launch" / "pixhawk_sensor_inputs.launch.py"
    ).read_text(encoding="utf-8")
    assert 'executable="pixhawk_sensor_adapter"' in launch
    assert 'default_value="/imu/data"' in launch
    assert 'default_value="/global_position/raw/fix"' in launch
    assert '"imu_output_topic": "/hardware/imu_primary/data"' in launch
    assert '"gnss_output_topic": "/hardware/gnss_primary/fix"' in launch
    assert '"output_topic": "/salus/imu/data"' in launch
    assert '"output_topic": "/salus/gps/fix"' in launch
    assert "controller" not in launch.lower()
    assert "vehicle_command" not in launch.lower()


def test_pixhawk_sensor_launch_supports_independent_expected_frames() -> None:
    launch = (
        Path(__file__).parents[1] / "launch" / "pixhawk_sensor_inputs.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument(\n            "imu_expected_frame"' in launch
    assert 'DeclareLaunchArgument(\n            "gnss_expected_frame"' in launch
    assert '"imu_expected_frame": imu_expected_frame' in launch
    assert '"gnss_expected_frame": gnss_expected_frame' in launch
    assert '"primary_frame": imu_expected_frame' in launch
    assert '"primary_frame": gnss_expected_frame' in launch


def test_real_observation_keeps_base_link_compatibility() -> None:
    observation = (
        Path(__file__).parents[1] / "launch" / "real_observation.launch.py"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("sensor_frame", default_value="base_link")' in observation
    assert '"sensor_frame": sensor_frame' in observation


def test_wrong_pixhawk_frames_remain_fail_closed() -> None:
    from sensor_msgs.msg import Imu, NavSatFix

    from salus_hardware.pixhawk_sensor_domain import validate_gnss, validate_imu

    imu = Imu()
    imu.header.frame_id = "gps_link"
    fix = NavSatFix()
    fix.header.frame_id = "imu_link"
    assert validate_imu(imu, expected_frame="imu_link") == "unexpected_frame"
    assert validate_gnss(fix, expected_frame="gps_link") == "unexpected_frame"
