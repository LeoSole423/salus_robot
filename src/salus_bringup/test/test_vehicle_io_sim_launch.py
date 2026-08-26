from pathlib import Path


ROOT = Path(__file__).parents[2]
BRINGUP_LAUNCH = Path(__file__).parents[1] / "launch" / "vehicle_io_sim.launch.py"
LOCALIZATION_LAUNCH = ROOT / "salus_localization" / "launch" / "localization_sim.launch.py"


def test_canonical_vehicle_io_uses_explicit_simulation_calibration() -> None:
    contents = BRINGUP_LAUNCH.read_text(encoding="utf-8")
    assert "legacy_drive_measurement_node" in contents
    assert "vehicle_kinematic_converter" in contents
    assert '"calibration_validated": True' in contents
    assert '"traction_linear_scale": 1.0' in contents
    assert '"steering_coefficients": [0.0, -1.0]' in contents


def test_localization_profiles_keep_one_main_odometry_authority() -> None:
    contents = LOCALIZATION_LAUNCH.read_text(encoding="utf-8")
    assert 'default_value="legacy"' in contents
    assert "legacy_condition" in contents
    assert "canonical_condition" in contents
    assert "kinematic_ackermann_odometry" in contents
    assert '"odom_topic": "/comparison/legacy/wheel_odometry"' in contents
    assert '"twist_topic": "/comparison/legacy/vehicle_twist"' in contents
    assert "legacy comparison is only valid with canonical odometry" in contents
