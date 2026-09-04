"""Synthetic runtime proof for the authoritative local real MVP (#179)."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from salus_interfaces.msg import DriveTelemetry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

from salus_localization.imu_normalizer import default_covariance


TELEMETRY_TOPIC = "/controller/drive_telemetry"
WHEEL_ODOMETRY_TOPIC = "/wheel/odometry"
IMU_TOPIC = "/salus/imu/data"
LOCAL_ODOMETRY_TOPIC = "/odometry/local"
LOCAL_NODE_NAME = "salus_local_ekf"


def _quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.w = math.cos(0.5 * yaw_rad)
    quaternion.z = math.sin(0.5 * yaw_rad)
    return quaternion


class RealLocalizationHarness(Node):
    """Publish typed DriveTelemetry + IMU samples and record all outputs."""

    def __init__(self) -> None:
        super().__init__("localization_local_real_runtime_probe")
        self.wheel_samples: list[Odometry] = []
        self.local_samples: list[Odometry] = []
        self.tf_messages: list[TFMessage] = []
        self.mode = "valid"
        self.first_invalid_stamp_s: float | None = None
        self.invalid_publish_count = 0
        self._speed_mps = 0.8
        self._steer_deg = -10.0  # The real profile inverts this to positive yaw.
        self._drive_publisher = self.create_publisher(
            DriveTelemetry, TELEMETRY_TOPIC, 10
        )
        self._imu_publisher = self.create_publisher(Imu, IMU_TOPIC, 10)
        self.create_subscription(Odometry, WHEEL_ODOMETRY_TOPIC, self._on_wheel, 50)
        self.create_subscription(Odometry, LOCAL_ODOMETRY_TOPIC, self._on_local, 50)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 50)
        self.create_timer(0.1, self._publish_inputs)

    def _publish_inputs(self) -> None:
        stamp = self.get_clock().now().to_msg()
        drive = DriveTelemetry()
        drive.stamp = stamp
        drive.ready = True
        drive.fresh = self.mode == "valid"
        drive.drive_enabled = True
        drive.estop = False
        drive.reverse_requested = False
        drive.speed_valid = self.mode != "invalid"
        drive.steer_valid = True
        drive.control_source = "synthetic_runtime"
        drive.speed_mps_measured = self._speed_mps
        drive.steer_deg_measured = self._steer_deg
        drive.brake_applied_pct = 0
        if self.mode == "invalid":
            self.invalid_publish_count += 1
            if self.first_invalid_stamp_s is None:
                self.first_invalid_stamp_s = (
                    float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
                )
        self._drive_publisher.publish(drive)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation = _quaternion_from_yaw(0.0)
        imu.orientation_covariance = default_covariance(0.01)
        imu.angular_velocity.z = 0.8 * math.tan(math.radians(10.0)) / 0.94
        imu.angular_velocity_covariance = default_covariance(0.01)
        imu.linear_acceleration_covariance = default_covariance(0.1)
        self._imu_publisher.publish(imu)

    def _on_wheel(self, message: Odometry) -> None:
        self.wheel_samples.append(message)

    def _on_local(self, message: Odometry) -> None:
        self.local_samples.append(message)

    def _on_tf(self, message: TFMessage) -> None:
        self.tf_messages.append(message)

    def publisher_names(self, topic: str) -> set[str]:
        return {info.node_name for info in self.get_publishers_info_by_topic(topic)}

    def wait_for(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        return False


def _start_launch(log_path: str) -> subprocess.Popen:
    return subprocess.Popen(
        [
            "ros2", "launch", "salus_localization",
            "localization_local_real.launch.py",
        ],
        stdout=open(log_path, "w", encoding="utf-8"),  # noqa: SIM115
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _stop(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:  # pragma: no cover - defensive cleanup
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=10)


def _stamp_seconds(message: Odometry) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _finite_odometry(message: Odometry) -> bool:
    pose = message.pose.pose
    twist = message.twist.twist
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.angular.z,
    )
    return all(math.isfinite(value) for value in values)


def test_real_localization_publishes_fresh_finite_outputs_and_one_tf_authority(
    tmp_path,
) -> None:
    rclpy.init()
    harness = RealLocalizationHarness()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    process = None
    log_path = tmp_path / "localization_local_real.log"
    try:
        process = _start_launch(str(log_path))
        assert harness.wait_for(
            lambda: {"ackermann_odometry", LOCAL_NODE_NAME}.issubset(
                set(harness.get_node_names())
            ),
            timeout_s=25.0,
        ), f"local real nodes never joined the graph; log:\n{log_path.read_text()[-3000:]}"
        assert harness.wait_for(
            lambda: LOCAL_NODE_NAME in harness.publisher_names(LOCAL_ODOMETRY_TOPIC),
            timeout_s=20.0,
        ), f"local EKF never advertised output; log:\n{log_path.read_text()[-3000:]}"
        assert harness.wait_for(
            lambda: len(harness.wheel_samples) >= 10
            and len(harness.local_samples) >= 10,
            timeout_s=20.0,
        ), f"local outputs missing; log:\n{log_path.read_text()[-3000:]}"
        assert harness.wait_for(
            lambda: any(message.transforms for message in harness.tf_messages),
            timeout_s=10.0,
        ), f"local EKF never published TF; log:\n{log_path.read_text()[-3000:]}"

        wheel = list(harness.wheel_samples)
        local = list(harness.local_samples)
        assert harness.publisher_names(WHEEL_ODOMETRY_TOPIC) == {"ackermann_odometry"}
        assert harness.publisher_names(LOCAL_ODOMETRY_TOPIC) == {LOCAL_NODE_NAME}
        assert harness.publisher_names("/tf") == {LOCAL_NODE_NAME}

        for samples in (wheel, local):
            assert all(_finite_odometry(message) for message in samples)
            stamps = [_stamp_seconds(message) for message in samples]
            assert stamps == sorted(stamps)
            assert stamps[-1] > stamps[0]
            assert all(message.header.frame_id == "odom" for message in samples)
            assert all(
                message.child_frame_id == "base_footprint" for message in samples
            )

        assert any(message.twist.twist.linear.x > 0.1 for message in wheel)
        assert any(message.twist.twist.angular.z > 0.01 for message in wheel)

        transforms = [
            transform
            for message in harness.tf_messages
            for transform in message.transforms
        ]
        assert transforms
        pairs = {(t.header.frame_id, t.child_frame_id) for t in transforms}
        assert pairs == {("odom", "base_footprint")}
        assert ("map", "odom") not in pairs
    finally:
        if process is not None:
            _stop(process)
        executor.remove_node(harness)
        harness.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_stale_or_invalid_drive_telemetry_does_not_invent_wheel_motion(tmp_path) -> None:
    rclpy.init()
    harness = RealLocalizationHarness()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    process = None
    log_path = tmp_path / "localization_local_real_stale.log"
    try:
        process = _start_launch(str(log_path))
        assert harness.wait_for(
            lambda: len(harness.wheel_samples) >= 12,
            timeout_s=25.0,
        ), f"wheel odometry did not start; log:\n{log_path.read_text()[-3000:]}"

        harness.mode = "stale"
        # Let the timer publish and DDS deliver stale samples before taking
        # the snapshot; otherwise one valid sample already queued at the mode
        # transition can make the assertion depend on scheduling.
        time.sleep(0.4)
        harness.wheel_samples.clear()
        assert harness.wait_for(
            lambda: len(harness.wheel_samples) >= 5,
            timeout_s=5.0,
        )
        stale_samples = list(harness.wheel_samples)
        first = stale_samples[0]
        assert all(_finite_odometry(message) for message in stale_samples)
        assert all(message.twist.twist.linear.x == 0.0 for message in stale_samples)
        assert all(message.twist.twist.angular.z == 0.0 for message in stale_samples)
        assert all(
            math.isclose(
                message.pose.pose.position.x,
                first.pose.pose.position.x,
                abs_tol=1.0e-9,
            )
            and math.isclose(
                message.pose.pose.position.y,
                first.pose.pose.position.y,
                abs_tol=1.0e-9,
            )
            for message in stale_samples
        )

        harness.wheel_samples.clear()
        harness.mode = "invalid"
        assert harness.wait_for(
            lambda: harness.first_invalid_stamp_s is not None
            and harness.invalid_publish_count >= 5,
            timeout_s=5.0,
        )
        invalid_boundary = harness.first_invalid_stamp_s
        assert invalid_boundary is not None
        assert all(
            _stamp_seconds(message) < invalid_boundary
            for message in harness.wheel_samples
        )
    finally:
        if process is not None:
            _stop(process)
        executor.remove_node(harness)
        harness.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
