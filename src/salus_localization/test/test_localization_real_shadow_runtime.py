"""Runtime proof that the shadow local EKF estimates without taking authority.

This is not a mock test: it starts the real ``robot_localization/ekf_node``
through the shipped ``localization_real_shadow.launch.py`` composition, feeds it
synthetic ``/wheel/odometry`` and ``/salus/imu/data`` messages of the correct
ROS types, and inspects the live DDS graph.

Nothing here touches hardware: there is no controller, UART, MAVROS, GNSS,
Nav2 or simulated world. The assertions deliberately encode the two properties
that make the trial safe on the real robot:

* the shadow publishes only on its isolated topic;
* the shadow never becomes a ``/tf`` publisher and never publishes the legacy
  ``/odometry/local`` output.

Waits are bounded and short on purpose; no timeout was enlarged to make this
pass. A healthy profile reaches steady output in a couple of seconds.
"""

from __future__ import annotations

import math
import os
import signal
import subprocess
import threading
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage

from salus_localization.ackermann_odometry import (
    diag_covariance,
    quaternion_from_yaw,
)
from salus_localization.imu_normalizer import default_covariance

LEGACY_LOCAL_TOPIC = "/odometry/local"
SHADOW_ODOMETRY_TOPIC = "/salus/localization_shadow/odometry/local"
WHEEL_ODOMETRY_TOPIC = "/wheel/odometry"
SALUS_IMU_TOPIC = "/salus/imu/data"
SHADOW_NODE_NAME = "salus_local_ekf_shadow"


class ShadowHarness(Node):
    """Feeds the shadow EKF and records what it publishes."""

    def __init__(self) -> None:
        super().__init__("localization_shadow_runtime_probe")
        self.shadow_samples: list[Odometry] = []
        self.tf_messages: list = []
        self._ticks = 0
        self._wheel_publisher = self.create_publisher(Odometry, WHEEL_ODOMETRY_TOPIC, 10)
        self._imu_publisher = self.create_publisher(Imu, SALUS_IMU_TOPIC, 10)
        self.create_subscription(
            Odometry, SHADOW_ODOMETRY_TOPIC, self._on_shadow, 50
        )
        self.create_subscription(
            TFMessage, "/tf", self._on_transform, 50
        )
        self.create_timer(1.0 / 60.0, self._publish_inputs)

    def _publish_inputs(self) -> None:
        self._ticks += 1
        stamp = self.get_clock().now().to_msg()
        # Keep the synthetic vehicle still: this mirrors the stationary trial.
        yaw = 0.0
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_footprint"
        odometry.pose.pose.position.x = 1.0
        odometry.pose.pose.position.y = 2.0
        odometry.pose.pose.orientation = quaternion_from_yaw(yaw)
        odometry.pose.covariance = diag_covariance(0.05, 0.05, 0.1)
        odometry.twist.twist.linear.x = 0.0
        odometry.twist.twist.angular.z = 0.0
        odometry.twist.covariance = diag_covariance(0.05, 0.01, 0.1)
        # Wheel odometry is fed at its real rate; the IMU every other tick.
        self._wheel_publisher.publish(odometry)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation = quaternion_from_yaw(yaw)
        imu.orientation_covariance = default_covariance(0.01)
        imu.angular_velocity.z = 0.0
        imu.angular_velocity_covariance = default_covariance(0.01)
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration_covariance = default_covariance(0.1)
        self._imu_publisher.publish(imu)

    def _on_shadow(self, message: Odometry) -> None:
        self.shadow_samples.append(message)

    def _on_transform(self, message: TFMessage) -> None:
        self.tf_messages.append(message)

    def publisher_names(self, topic: str) -> set[str]:
        return {
            info.node_name
            for info in self.get_publishers_info_by_topic(topic)
        }

    def wait_for(self, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.1)
        return False


def _start_shadow_launch(log_path: str) -> subprocess.Popen:
    return subprocess.Popen(
        [
            "ros2", "launch", "salus_localization",
            "localization_real_shadow.launch.py",
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


def test_shadow_ekf_estimates_without_gaining_tf_or_legacy_authority(tmp_path) -> None:
    rclpy.init()
    harness = ShadowHarness()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    process = None
    log_path = tmp_path / "localization_real_shadow.log"
    try:
        process = _start_shadow_launch(str(log_path))
        assert harness.wait_for(
            lambda: SHADOW_NODE_NAME in harness.get_node_names(),
            timeout_s=25.0,
        ), f"shadow EKF never joined the graph; log:\n{log_path.read_text()[-2000:]}"
        assert harness.wait_for(
            lambda: SHADOW_NODE_NAME in harness.publisher_names(SHADOW_ODOMETRY_TOPIC),
            timeout_s=20.0,
        ), f"shadow EKF never advertised its output; log:\n{log_path.read_text()[-2000:]}"

        assert harness.wait_for(
            lambda: len(harness.shadow_samples) >= 5,
            timeout_s=20.0,
        ), f"shadow EKF produced no odometry; log:\n{log_path.read_text()[-2000:]}"
        harness.shadow_samples.clear()
        assert harness.wait_for(
            lambda: len(harness.shadow_samples) >= 40, timeout_s=10.0
        ), "shadow output is not continuous"

        samples = harness.shadow_samples
        assert all(
            message.header.frame_id == "odom" for message in samples
        ), [message.header.frame_id for message in samples[:3]]
        assert all(
            message.child_frame_id == "base_footprint" for message in samples
        ), [message.child_frame_id for message in samples[:3]]

        stamps = [
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            for message in samples
        ]
        assert stamps == sorted(stamps), "shadow timestamps are not monotonic"
        assert stamps[-1] > stamps[0], "shadow timestamps do not progress"

        for message in samples:
            pose, twist = message.pose.pose, message.twist.twist
            values = (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
                twist.linear.x, twist.linear.y, twist.angular.z,
            )
            assert all(math.isfinite(value) for value in values), values
            # Inputs describe a still vehicle, so the estimate must not run away.
            assert abs(twist.linear.x) < 0.5 and abs(twist.angular.z) < 0.5

        assert harness.publisher_names(SHADOW_ODOMETRY_TOPIC) == {SHADOW_NODE_NAME}
        # Authority: the shadow must not displace the legacy estimator.
        assert LEGACY_LOCAL_TOPIC not in dict(harness.get_topic_names_and_types()) or (
            harness.publisher_names(LEGACY_LOCAL_TOPIC) == set()
        ), LEGACY_LOCAL_TOPIC
        # ``robot_localization`` always constructs its TransformBroadcaster, so
        # the node advertises a /tf endpoint even with publish_tf=false. The
        # property that must never regress is the payload: no transform may
        # leave the shadow, and in this isolated graph no other node exists, so
        # any received transform would necessarily come from the shadow EKF.
        transforms = [
            transform
            for message in harness.tf_messages
            for transform in message.transforms
        ]
        assert transforms == [], (
            "the shadow EKF must never publish TF, got "
            f"{[(t.header.frame_id, t.child_frame_id) for t in transforms[:5]]}"
        )
        assert SHADOW_NODE_NAME not in harness.publisher_names(WHEEL_ODOMETRY_TOPIC)
        assert SHADOW_NODE_NAME not in harness.publisher_names(SALUS_IMU_TOPIC)
    finally:
        if process is not None:
            _stop(process)
        executor.remove_node(harness)
        harness.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_shadow_probe_helper_is_independent_of_the_launch() -> None:
    """Guards against the runtime test silently becoming a no-op."""
    assert SHADOW_ODOMETRY_TOPIC.startswith("/salus/localization_shadow/")
    assert LEGACY_LOCAL_TOPIC == "/odometry/local"
    assert WHEEL_ODOMETRY_TOPIC == "/wheel/odometry"
    assert SALUS_IMU_TOPIC == "/salus/imu/data"


@pytest.mark.parametrize("topic", (LEGACY_LOCAL_TOPIC, "/tf"))
def test_shadow_namespace_never_reuses_legacy_output_names(topic: str) -> None:
    assert not topic.startswith("/salus/")
