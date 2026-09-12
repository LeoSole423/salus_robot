"""Static contract checks for issue #64 simulation sensor profiles."""

from pathlib import Path

import pytest
import yaml


PROFILE_DIR = Path(__file__).parents[1] / "config" / "sensor_profiles"
PROFILE_IDS = ("clean", "independent_nominal", "degraded")
SENSOR_KEYS = ("imu", "wheel", "gnss")


@pytest.mark.parametrize("profile_id", PROFILE_IDS)
def test_profile_is_complete_and_self_identifying(profile_id):
    path = PROFILE_DIR / f"{profile_id}.yaml"
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert data["schema_version"] == 1
    assert data["id"] == profile_id
    assert tuple(data) == ("schema_version", "id", "description", "imu", "wheel", "gnss")
    assert tuple(data["imu"]) == (
        "gyro_bias_rad_s", "gyro_noise_stddev_rad_s", "latency_s", "jitter_s",
        "dropout_probability", "stale_after_s",
    )
    assert tuple(data["wheel"]) == (
        "traction_scale", "speed_noise_stddev_mps", "steering_bias_deg",
        "steering_noise_stddev_deg", "latency_s", "jitter_s",
        "dropout_probability", "stale_after_s",
    )
    assert tuple(data["gnss"]) == (
        "horizontal_noise_stddev_m", "vertical_noise_stddev_m", "latency_s",
        "jitter_s", "dropout_probability", "stale_after_s", "quality_states",
        "quality_dwell_s",
    )


def test_clean_profile_has_no_perturbation():
    data = yaml.safe_load((PROFILE_DIR / "clean.yaml").read_text(encoding="utf-8"))
    for sensor in SENSOR_KEYS:
        assert data[sensor]["latency_s"] == 0.0
        assert data[sensor]["jitter_s"] == 0.0
        assert data[sensor]["dropout_probability"] == 0.0


def test_independent_nominal_yaml_matches_the_frozen_contract():
    data = yaml.safe_load(
        (PROFILE_DIR / "independent_nominal.yaml").read_text(encoding="utf-8")
    )
    assert data["imu"]["gyro_bias_rad_s"] == pytest.approx(0.002)
    assert data["imu"]["gyro_noise_stddev_rad_s"] == pytest.approx(0.005)
    assert data["imu"]["dropout_probability"] == pytest.approx(0.002)
    assert data["wheel"]["traction_scale"] == pytest.approx(1.005)
    assert data["wheel"]["steering_bias_deg"] == pytest.approx(0.15)
    assert data["wheel"]["steering_noise_stddev_deg"] == pytest.approx(0.20)
    assert data["gnss"]["horizontal_noise_stddev_m"] == pytest.approx(0.02)
    assert data["gnss"]["vertical_noise_stddev_m"] == pytest.approx(0.04)
    assert data["gnss"]["jitter_s"] == pytest.approx(0.03)
