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
