"""Regression tests for the isolated physical RS16 owner."""

import subprocess
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).parents[1]
REPOSITORY_ROOT = ROOT.parents[1]
DEPENDENCIES = REPOSITORY_ROOT / "dependencies.repos"
CONFIG = ROOT / "config/rs16.yaml"
LAUNCH = ROOT / "launch/rs16_real.launch.py"
MOUNTS = ROOT.parent / "salus_description/urdf/components/sensor_mounts.xacro"


SDK_COMMIT = "7c4ea25fada93442c3d390aa4ef05e240999b851"
MSG_COMMIT = "fe8a95cb242bd294cc3d5e3422f2093fb49a56ee"
DRIVER_COMMIT = "cd358851ab65bf57fc7e321837be2a425305b298"


def test_dependencies_pin_sdk_and_message_repositories() -> None:
    document = yaml.safe_load(DEPENDENCIES.read_text(encoding="utf-8"))
    repositories = document["repositories"]

    assert repositories["src/rslidar_sdk"] == {
        "type": "git",
        "url": "https://github.com/RoboSense-LiDAR/rslidar_sdk.git",
        "version": SDK_COMMIT,
    }
    assert repositories["src/rslidar_msg"] == {
        "type": "git",
        "url": "https://github.com/RoboSense-LiDAR/rslidar_msg.git",
        "version": MSG_COMMIT,
    }


def test_initialized_sources_match_pins_when_imported() -> None:
    sdk = REPOSITORY_ROOT / "src/rslidar_sdk"
    msg = REPOSITORY_ROOT / "src/rslidar_msg"
    if not (sdk / ".git").exists() or not (msg / ".git").exists():
        pytest.skip("external repositories are imported by the workspace validator")

    assert subprocess.check_output(
        ["git", "-C", str(sdk), "rev-parse", "HEAD"], text=True
    ).strip() == SDK_COMMIT
    assert subprocess.check_output(
        ["git", "-C", str(msg), "rev-parse", "HEAD"], text=True
    ).strip() == MSG_COMMIT
    driver = sdk / "src/rs_driver"
    assert subprocess.check_output(
        ["git", "-C", str(driver), "rev-parse", "HEAD"], text=True
    ).strip() == DRIVER_COMMIT


def test_rs16_config_preserves_the_observed_physical_profile() -> None:
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert document["common"] == {
        "msg_source": 1,
        "send_packet_ros": False,
        "send_point_cloud_ros": True,
    }
    driver = document["lidar"][0]["driver"]
    assert driver == {
        "lidar_type": "RS16",
        "msop_port": 6699,
        "difop_port": 7788,
        "host_address": "0.0.0.0",
        "group_address": "0.0.0.0",
        "imu_port": 0,
        "user_layer_bytes": 0,
        "tail_layer_bytes": 0,
        "min_distance": 0.4,
        "max_distance": 20,
        "use_lidar_clock": False,
        "dense_points": True,
        "ts_first_point": True,
        "start_angle": 270,
        "end_angle": 90,
        "pcap_repeat": True,
        "pcap_rate": 1.0,
        "pcap_path": "/home/ros/lidar.pcap",
    }
    assert document["lidar"][0]["ros"] == {
        "ros_frame_id": "lidar_link",
        "ros_recv_packet_topic": "/rslidar_packets",
        "ros_send_packet_topic": "/rslidar_packets",
        "ros_send_imu_data_topic": "/rslidar_imu_data",
        "ros_send_point_cloud_topic": "/scan_3d",
        "ros_queue_length": 30,
    }


def test_rs16_real_launch_has_one_raw_owner_and_one_argument() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")

    assert contents.count("DeclareLaunchArgument(") == 1
    assert contents.count("Node(") == 1
    assert contents.count("package=\"rslidar_sdk\"") == 1
    assert contents.count("executable=\"rslidar_sdk_node\"") == 1
    assert 'name="rslidar_sdk_node"' not in contents
    assert 'parameters=[{"config_path": config_path}]' in contents
    assert "config_path" in contents


def test_rs16_real_launch_has_no_other_runtime_authorities() -> None:
    contents = LAUNCH.read_text(encoding="utf-8").lower()
    for forbidden in (
        "rviz",
        "cloud_normalizer",
        "pointcloud_to_laserscan",
        "nav2",
        "ntrip",
        "rtcm",
        "uart",
        "mavros",
        "robot_state_publisher",
    ):
        assert forbidden not in contents
    assert '"/scan_3d"' not in contents


def test_lidar_mount_remains_the_legacy_physical_geometry() -> None:
    contents = MOUNTS.read_text(encoding="utf-8")
    assert (
        'name="lidar_link" xyz="0.92 0 0.65" '
        'rpy="0 0.1745 0"'
    ) in contents


def test_real_observation_does_not_start_rs16() -> None:
    observation = (
        ROOT.parent / "salus_bringup/launch/real_observation.launch.py"
    ).read_text(encoding="utf-8").lower()
    assert "rslidar" not in observation
    assert "rs16" not in observation
