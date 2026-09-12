"""Static contract checks for issue #64 simulation sensor profiles."""

from pathlib import Path

import pytest
import yaml


PROFILE_DIR = Path(__file__).parents[1] / "config" / "sensor_profiles"
PROFILE_IDS = ("clean", "independent_nominal", "degraded")
NODE_KEYS = ("sim_drive_sensor_adapter", "sim_imu_from_odom", "sim_gps_normalizer")


@pytest.mark.parametrize("profile_id", PROFILE_IDS)
def test_profile_is_a_ros_parameter_file(profile_id):
    path = PROFILE_DIR / f"{profile_id}.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert tuple(data) == NODE_KEYS
    assert all("ros__parameters" in data[node] for node in NODE_KEYS)
    assert all(
        "stale_after_s" not in key
        for node in NODE_KEYS
        for key in data[node]["ros__parameters"]
    )
    for node in NODE_KEYS:
        assert data[node]["ros__parameters"]["sim_sensor_profile"] == profile_id


def test_clean_profile_has_no_perturbation():
    data = yaml.safe_load((PROFILE_DIR / "clean.yaml").read_text(encoding="utf-8"))
    for node, keys in {
        "sim_drive_sensor_adapter": (
            "wheel.latency_s",
            "wheel.jitter_s",
            "wheel.dropout_probability",
        ),
        "sim_imu_from_odom": (
            "imu.latency_s",
            "imu.jitter_s",
            "imu.dropout_probability",
        ),
        "sim_gps_normalizer": (
            "gnss.latency_s",
            "gnss.jitter_s",
            "gnss.dropout_probability",
        ),
    }.items():
        params = data[node]["ros__parameters"]
        assert all(params[key] == 0.0 for key in keys)


def test_independent_nominal_yaml_matches_the_frozen_contract():
    data = yaml.safe_load(
        (PROFILE_DIR / "independent_nominal.yaml").read_text(encoding="utf-8")
    )
    expected = {
        ("sim_imu_from_odom", "imu.gyro_bias_rad_s"): 0.002,
        ("sim_imu_from_odom", "imu.gyro_noise_stddev_rad_s"): 0.005,
        ("sim_imu_from_odom", "imu.dropout_probability"): 0.002,
        ("sim_drive_sensor_adapter", "wheel.traction_scale"): 1.005,
        ("sim_drive_sensor_adapter", "steering.bias_deg"): 0.15,
        ("sim_drive_sensor_adapter", "steering.noise_stddev_deg"): 0.20,
        ("sim_gps_normalizer", "gnss.noise_stddev_m"): 0.02,
        ("sim_gps_normalizer", "gnss.vertical_noise_stddev_m"): 0.04,
        ("sim_gps_normalizer", "gnss.jitter_s"): 0.03,
    }
    for (node, parameter), value in expected.items():
        assert data[node]["ros__parameters"][parameter] == pytest.approx(value)
