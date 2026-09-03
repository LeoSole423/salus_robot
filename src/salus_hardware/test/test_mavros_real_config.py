import subprocess
from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = ROOT.parents[1]
PLUGINLISTS = ROOT / "config/mavros_sensor_only_pluginlists.yaml"
OVERRIDES = ROOT / "config/mavros_apm_overrides.yaml"


def test_sensor_only_allowlist_matches_the_effective_legacy_profile() -> None:
    config = yaml.safe_load(PLUGINLISTS.read_text(encoding="utf-8"))

    assert config == {
        "/**": {
            "ros__parameters": {
                "plugin_allowlist": [
                    "sys_status",
                    "sys_time",
                    "imu",
                    "global_position",
                    "local_position",
                    "gps_status",
                    "gps_rtk",
                ]
            }
        }
    }


def test_mavros_overrides_keep_all_tf_publication_disabled() -> None:
    config = yaml.safe_load(OVERRIDES.read_text(encoding="utf-8"))

    assert config["/**"]["ros__parameters"]["startup_px4_usb_quirk"] is False
    assert config["/**/global_position"]["ros__parameters"] == {
        "frame_id": "gps_link",
        "child_frame_id": "base_footprint",
        "tf.send": False,
    }
    assert config["/**/imu"]["ros__parameters"] == {"frame_id": "imu_link"}
    assert config["/**/local_position"]["ros__parameters"] == {
        "frame_id": "odom",
        "tf.send": False,
        "tf.frame_id": "odom",
        "tf.child_frame_id": "base_footprint",
        "tf.send_fcu": False,
    }


def test_image_recipe_installs_mavros_extras_and_geographiclib_data() -> None:
    dockerfile = (REPOSITORY_ROOT / "Dockerfile").read_text(encoding="utf-8")

    assert "ros-humble-mavros" in dockerfile
    assert "ros-humble-mavros-extras" in dockerfile
    assert "geographiclib-tools" in dockerfile
    assert "ros2 run mavros install_geographiclib_datasets.sh" in dockerfile


def test_built_image_exposes_mavros_and_geographiclib_datasets() -> None:
    for package in ("mavros", "mavros_extras"):
        completed = subprocess.run(
            ["ros2", "pkg", "prefix", package],
            check=False,
            capture_output=True,
            text=True,
        )
        assert completed.returncode == 0, completed.stderr

    geographiclib_root = Path("/usr/share/GeographicLib")
    assert any(
        path.suffix == ".pgm" for path in geographiclib_root.rglob("*")
    ), "MAVROS GeographicLib datasets are missing from the image"
