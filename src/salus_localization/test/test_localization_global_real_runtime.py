"""Isolated synthetic runtime proof for the real global profile."""

from __future__ import annotations

import math
import os

# This must be set before rclpy.init(); the launch child inherits the same DDS
# domain and cannot observe or affect another test/runtime.
os.environ["ROS_DOMAIN_ID"] = str(190 + (os.getpid() % 40))

import signal
import subprocess
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import FromLL
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from salus_interfaces.msg import DriveTelemetry, GnssRtkStatus
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster


DATUM_LAT = -31.4859026607927
DATUM_LON = -64.24097358249034
RUNTIME_NODE = "global_localization_real_runtime_probe"


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _finite_message(message: Odometry) -> bool:
    values = (
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        message.twist.twist.linear.x,
        message.twist.twist.angular.z,
    )
    return all(math.isfinite(value) for value in values)


class GlobalLocalizationRuntimeProbe(Node):
    def __init__(self) -> None:
        super().__init__(RUNTIME_NODE)
        self._tick = 0
        self.rtk_quality = GnssRtkStatus.RTK_FIXED
        self.corrections_fresh = True
        self.heading: list[Imu] = []
        self.orientation: list[Imu] = []
        self.gps_odom: list[Odometry] = []
        self.global_odom: list[Odometry] = []
        self.tf_messages: list[TFMessage] = []
        self._odom_pub = self.create_publisher(Odometry, "/odometry/local", 10)
        self._imu_pub = self.create_publisher(Imu, "/salus/imu/data", 10)
        self._gps_pub = self.create_publisher(NavSatFix, "/salus/gps/fix", 10)
        self._drive_pub = self.create_publisher(
            DriveTelemetry, "/controller/drive_telemetry", 10
        )
        self._rtk_pub = self.create_publisher(
            GnssRtkStatus,
            "/salus/hardware/gnss_primary/rtk_status",
            10,
        )
        self.create_subscription(Imu, "/gps/course_heading", self.heading.append, 10)
        self.create_subscription(
            Imu, "/localization/orientation", self.orientation.append, 10
        )
        self.create_subscription(
            Odometry, "/odometry/gps", self.gps_odom.append, 10
        )
        self.create_subscription(
            Odometry, "/odometry/global", self.global_odom.append, 10
        )
        self.create_subscription(TFMessage, "/tf", self.tf_messages.append, 50)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.from_ll = self.create_client(FromLL, "/fromLL")
        self.create_timer(0.05, self._publish_inputs)

    def _publish_inputs(self) -> None:
        self._tick += 1
        stamp = self.get_clock().now().to_msg()
        elapsed = self._tick * 0.05
        longitude = DATUM_LON + elapsed / (
            111320.0 * math.cos(math.radians(DATUM_LAT))
        )

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_footprint"
        odometry.pose.pose.orientation.w = 1.0
        odometry.pose.pose.position.x = elapsed
        odometry.twist.twist.linear.x = 1.0
        odometry.twist.twist.angular.z = 0.0
        self._odom_pub.publish(odometry)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = 0.0
        self._imu_pub.publish(imu)

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "base_footprint"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.latitude = DATUM_LAT
        fix.longitude = longitude
        fix.altitude = 0.0
        self._gps_pub.publish(fix)

        drive = DriveTelemetry()
        drive.stamp = stamp
        drive.fresh = True
        drive.speed_valid = True
        drive.steer_valid = True
        drive.speed_mps_measured = 1.0
        drive.steer_deg_measured = 0.0
        self._drive_pub.publish(drive)

        status = GnssRtkStatus()
        status.header.stamp = stamp
        status.fix_quality = self.rtk_quality
        status.corrections_fresh = self.corrections_fresh
        self._rtk_pub.publish(status)

        for parent, child, x, z in (
            ("odom", "base_footprint", elapsed, 0.0),
            ("base_footprint", "base_link", 0.0, 0.0),
        ):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x = x
            transform.transform.translation.z = z
            transform.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(transform)

    def wait_for(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.05)
        return False


def _start_launch(log_path) -> tuple[subprocess.Popen, object]:
    log_handle = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        [
            "ros2",
            "launch",
            "salus_localization",
            "global_localization_real.launch.py",
        ],
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
        start_new_session=True,
    )
    return process, log_handle


def _stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:  # pragma: no cover
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=10)


def test_real_global_profile_runtime_isolated_and_rtk_gated(tmp_path) -> None:
    rclpy.init()
    probe = GlobalLocalizationRuntimeProbe()
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    process = None
    log_path = tmp_path / "global_localization_real.log"
    log_handle = None
    try:
        process, log_handle = _start_launch(log_path)
        assert probe.wait_for(
            lambda: "salus_global_ekf" in probe.get_node_names(), 25.0
        ), f"global EKF never joined the graph; log:\n{log_path.read_text()[-3000:]}"
        assert probe.wait_for(lambda: len(probe.heading) >= 1, 25.0), (
            "course heading did not appear with RTK_FIXED and straight motion; "
            f"log:\n{log_path.read_text()[-3000:]}"
        )
        assert probe.wait_for(lambda: len(probe.orientation) >= 1, 10.0)
        assert probe.wait_for(lambda: len(probe.gps_odom) >= 3, 20.0)
        assert probe.wait_for(lambda: len(probe.global_odom) >= 3, 20.0)
        assert probe.wait_for(
            lambda: any(
                transform.header.frame_id == "map"
                and transform.child_frame_id == "odom"
                for message in probe.tf_messages
                for transform in message.transforms
            ),
            20.0,
        )

        for samples in (
            probe.heading,
            probe.orientation,
            probe.gps_odom,
            probe.global_odom,
        ):
            stamps = [_stamp_seconds(message.header.stamp) for message in samples]
            assert all(math.isfinite(value) for value in stamps)
            assert all(
                newer > older for older, newer in zip(stamps, stamps[1:])
            )
            assert all(
                _finite_message(message)
                for message in samples
                if isinstance(message, Odometry)
            )
        assert probe.heading[-1].header.frame_id == "base_footprint"
        assert probe.orientation[-1].header.frame_id == "base_footprint"
        assert probe.gps_odom[-1].header.frame_id == "map"
        assert probe.global_odom[-1].header.frame_id == "map"
        assert probe.global_odom[-1].child_frame_id == "base_footprint"

        tf_publishers = {
            info.node_name for info in probe.get_publishers_info_by_topic("/tf")
        }
        assert "salus_global_ekf" in tf_publishers
        assert RUNTIME_NODE in tf_publishers
        assert "salus_local_ekf" not in tf_publishers
        assert "ackermann_odometry" not in probe.get_node_names()

        assert probe.wait_for(lambda: probe.from_ll.service_is_ready(), 10.0)
        request = FromLL.Request()
        request.ll_point.latitude = DATUM_LAT
        request.ll_point.longitude = DATUM_LON
        request.ll_point.altitude = 0.0
        future = probe.from_ll.call_async(request)
        assert probe.wait_for(lambda: future.done(), 10.0)
        response = future.result()
        assert response is not None
        assert math.hypot(response.map_point.x, response.map_point.y) < 0.1

        service_names = {name for name, _ in probe.get_service_names_and_types()}
        assert "/fromLL" in service_names
        assert "/navsat_transform/fromLL" not in service_names

        probe.rtk_quality = GnssRtkStatus.RTK_FLOAT
        probe.corrections_fresh = True
        time.sleep(0.5)  # allow the typed status to cross DDS
        probe.heading.clear()
        probe.orientation.clear()
        time.sleep(1.2)
        assert probe.heading == []
        assert probe.orientation == []

        probe.rtk_quality = GnssRtkStatus.UNKNOWN
        time.sleep(0.5)
        probe.heading.clear()
        probe.orientation.clear()
        time.sleep(1.2)
        assert probe.heading == []
        assert probe.orientation == []
    finally:
        _stop(process)
        if log_handle is not None:
            log_handle.close()
        executor.shutdown()
        probe.destroy_node()
        spinner.join(timeout=5)
        if rclpy.ok():
            rclpy.shutdown()
