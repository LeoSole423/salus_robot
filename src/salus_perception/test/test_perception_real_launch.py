"""Structural and synthetic runtime checks for the real perception profile."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


ROOT = Path(__file__).parents[1]
LAUNCH = ROOT / "launch" / "perception_real.launch.py"


def test_real_perception_launch_has_exactly_three_nodes() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert contents.count("Node(") == 3
    assert contents.count('package="salus_perception"') == 2
    assert 'package="pointcloud_to_laserscan"' in contents
    assert 'executable="scan_ground_filter"' in contents
    assert 'executable="pointcloud_to_laserscan_node"' in contents
    assert 'executable="scan_noise_filter"' in contents


def test_real_perception_launch_preserves_raw_boundaries_and_forbids_authorities() -> None:
    contents = LAUNCH.read_text(encoding="utf-8").lower()
    assert '"input_topic": "/scan_3d"' in contents
    assert '"output_topic": "/obstacles_cloud"' in contents
    assert '"output_topic": "/scan_clean"' in contents
    assert "cloud_normalizer" not in contents
    assert '"/scan_3d"' not in contents.split("pointcloud_to_laserscan", 1)[1]
    for forbidden in (
        "rslidar",
        "robot_state_publisher",
        "nav2",
        "collision_monitor",
        "uart",
        "mavros",
        "ntrip",
        '"/tf"',
    ):
        assert forbidden not in contents


def test_real_perception_launch_fixes_the_184_parameters() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    for expected in (
        '"target_frame": "base_footprint"',
        '"wheelbase_m": 0.94',
        '"profile": "urban"',
        '"ground_tolerance_m": 0.20',
        '"range_max": 20.0',
        '"transform_tolerance": 0.1',
        '"min_height": -0.1',
        '"max_height": 1.6',
        '"angle_min": -1.5707963',
        '"angle_max": 1.5707963',
        '"angle_increment": 0.00872665',
        '"range_min": 0.4',
        '"use_inf": True',
        '"speckle_window": 2',
        '"speckle_max_range": 12.0',
        '"max_deviation_m": 0.30',
        '"use_sim_time": False',
    ):
        assert expected in contents


class PerceptionRuntimeHarness(Node):
    """Publish a cloud and fixture TF, then collect every pipeline boundary."""

    def __init__(self, *, publish_tf: bool) -> None:
        super().__init__("perception_real_runtime_probe")
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/scan_3d", qos_profile_sensor_data
        )
        self.obstacle_clouds: list[PointCloud2] = []
        self.scans: list[LaserScan] = []
        self.clean_scans: list[LaserScan] = []
        self._publish_tf = publish_tf
        self._tf = StaticTransformBroadcaster(self) if publish_tf else None
        self.create_subscription(
            PointCloud2,
            "/obstacles_cloud",
            self.obstacle_clouds.append,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/scan", self.scans.append, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan_clean", self.clean_scans.append, qos_profile_sensor_data
        )

    def publish_fixture_tf(self) -> None:
        if self._tf is None:
            return
        base = TransformStamped()
        base.header.stamp = self.get_clock().now().to_msg()
        base.header.frame_id = "base_footprint"
        base.child_frame_id = "base_link"
        base.transform.rotation.w = 1.0
        lidar = TransformStamped()
        lidar.header.stamp = base.header.stamp
        lidar.header.frame_id = "base_link"
        lidar.child_frame_id = "lidar_link"
        lidar.transform.translation.x = 0.92
        lidar.transform.translation.z = 0.65
        lidar.transform.rotation.y = math.sin(0.1745 / 2.0)
        lidar.transform.rotation.w = math.cos(0.1745 / 2.0)
        self._tf.sendTransform([base, lidar])

    def publish_cloud(self, frame_id: str = "lidar_link") -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        # Build points in lidar_link from their desired post-TF coordinates.
        # The physical static TF is base_link -> lidar_link; the adapter applies
        # the lookup result to the cloud, so invert that rigid transform here.
        ground = [
            self._lidar_point_for_output(x, y, z)
            for x, y, z in ((1.5, -0.1, 0.0), (2.0, 0.0, 0.0), (2.5, 0.1, 0.1))
        ]
        obstacle = [
            self._lidar_point_for_output(2.0, y, 0.8)
            for y in (-0.04, -0.02, 0.0, 0.02, 0.04)
        ]
        self.cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, ground + obstacle))

    @staticmethod
    def _lidar_point_for_output(x: float, y: float, z: float) -> tuple[float, float, float]:
        """Invert the frozen transform so the node output is (x, y, z)."""
        pitch = 0.1745
        dx, dz = x - 0.92, z - 0.65
        return (
            math.cos(pitch) * dx - math.sin(pitch) * dz,
            y,
            math.sin(pitch) * dx + math.cos(pitch) * dz,
        )

    def wait_for(self, predicate, timeout_s: float = 20.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return False


def _start_launch(log_path: Path) -> subprocess.Popen:
    stream = log_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        ["ros2", "launch", "salus_perception", "perception_real.launch.py"],
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _stop_launch(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:  # pragma: no cover - defensive cleanup
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=10)


def _run_runtime_probe(tmp_path: Path, *, publish_tf: bool) -> PerceptionRuntimeHarness:
    rclpy.init()
    harness = PerceptionRuntimeHarness(publish_tf=publish_tf)
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    process = _start_launch(tmp_path / "perception_real.log")
    harness._launch_process = process  # type: ignore[attr-defined]
    harness._executor = executor  # type: ignore[attr-defined]
    harness._spinner = spinner  # type: ignore[attr-defined]
    return harness


def _finish_runtime_probe(harness: PerceptionRuntimeHarness) -> None:
    process = harness._launch_process  # type: ignore[attr-defined]
    _stop_launch(process)
    harness._executor.remove_node(harness)  # type: ignore[attr-defined]
    harness.destroy_node()
    harness._executor.shutdown()  # type: ignore[attr-defined]
    rclpy.shutdown()


def test_synthetic_cloud_produces_fresh_plausible_clean_scan(tmp_path: Path) -> None:
    harness = _run_runtime_probe(tmp_path, publish_tf=True)
    try:
        assert harness.wait_for(
            lambda: {
                "scan_ground_filter",
                "pointcloud_to_laserscan",
                "scan_noise_filter",
            }.issubset(set(harness.get_node_names())),
        )
        harness.publish_fixture_tf()
        for _ in range(8):
            harness.publish_fixture_tf()
            harness.publish_cloud()
            time.sleep(0.1)
        assert harness.wait_for(
            lambda: len(harness.obstacle_clouds) >= 3
            and len(harness.scans) >= 3
            and len(harness.clean_scans) >= 3,
        )
        assert all(message.header.frame_id == "base_footprint" for message in harness.obstacle_clouds)
        assert all(message.header.frame_id == "base_footprint" for message in harness.clean_scans)
        output_points = list(
            point_cloud2.read_points(
                harness.obstacle_clouds[-1], field_names=("x", "y", "z"), skip_nans=False
            )
        )
        assert output_points
        assert all(point[2] > 0.20 for point in output_points)
        assert any(math.isclose(point[0], 2.0, abs_tol=0.15) for point in output_points)
        clean = harness.clean_scans[-1]
        finite = [value for value in clean.ranges if math.isfinite(value)]
        assert finite
        assert min(abs(value - 2.0) for value in finite) <= 0.15
        stamps = [
            message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
            for message in harness.clean_scans
        ]
        assert stamps == sorted(stamps)
        assert stamps[-1] > stamps[0]
    finally:
        _finish_runtime_probe(harness)


def test_missing_tf_fails_closed_without_scan_output(tmp_path: Path) -> None:
    harness = _run_runtime_probe(tmp_path, publish_tf=False)
    try:
        assert harness.wait_for(
            lambda: "scan_ground_filter" in set(harness.get_node_names())
        )
        harness.publish_cloud(frame_id="missing_lidar_frame")
        time.sleep(1.5)
        assert harness.obstacle_clouds == []
        assert harness.scans == []
        assert harness.clean_scans == []
    finally:
        _finish_runtime_probe(harness)
