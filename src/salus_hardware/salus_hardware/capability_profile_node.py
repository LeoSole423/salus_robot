"""Publish the explicitly selected capability profile as a typed snapshot."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from salus_interfaces.msg import CapabilityState, SystemCapabilities
from sensor_msgs.msg import Imu

from .capability_profile import declarations_for_profile, normalize_profile, observed_state


class CapabilityProfileNode(Node):
    """Latched declaration; it never changes profiles in response to faults."""

    def __init__(self) -> None:
        super().__init__("capability_profile")
        self.declare_parameter("profile", "obstacle_detection")
        self.declare_parameter("imu_source", "imu_primary")
        self.declare_parameter("orientation_source", "course_over_ground")
        self.declare_parameter("output_topic", "/system/capabilities")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("orientation_topic", "/localization/orientation")
        self.declare_parameter("imu_timeout_s", 0.5)
        self.declare_parameter("orientation_timeout_s", 1.0)
        self._profile = normalize_profile(self.get_parameter("profile").value)
        self._last_sample_s = {
            "local_motion_imu": None,
            "global_orientation": None,
        }
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            SystemCapabilities, str(self.get_parameter("output_topic").value), qos
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            lambda _message: self._record_sample("local_motion_imu"),
            10,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("orientation_topic").value),
            lambda _message: self._record_sample("global_orientation"),
            10,
        )
        self.create_timer(1.0, self._publish)
        self._publish()

    def _record_sample(self, capability_id: str) -> None:
        self._last_sample_s[capability_id] = time.monotonic()

    def _observed_sensor_state(self, capability_id: str) -> int:
        timeout_parameter = (
            "imu_timeout_s"
            if capability_id == "local_motion_imu"
            else "orientation_timeout_s"
        )
        return observed_state(
            now_s=time.monotonic(),
            last_sample_s=self._last_sample_s[capability_id],
            timeout_s=float(self.get_parameter(timeout_parameter).value),
            unavailable_state=CapabilityState.STATE_UNAVAILABLE,
            stale_state=CapabilityState.STATE_STALE,
            ready_state=CapabilityState.STATE_READY,
        )

    def _publish(self) -> None:
        message = SystemCapabilities()
        message.header.stamp = self.get_clock().now().to_msg()
        message.profile = self._profile
        for item in declarations_for_profile(
            self._profile,
            ready_state=CapabilityState.STATE_ENABLED_BY_PROFILE,
            disabled_state=CapabilityState.STATE_DISABLED_BY_PROFILE,
            imu_source=self.get_parameter("imu_source").value,
            orientation_source=self.get_parameter("orientation_source").value,
        ):
            capability = CapabilityState()
            capability.capability_id = item.capability_id
            capability.state = (
                self._observed_sensor_state(item.capability_id)
                if item.capability_id in self._last_sample_s
                else item.state
            )
            capability.required = item.required
            capability.enabled = item.enabled
            capability.source_ids = list(item.source_ids)
            capability.detail = item.detail
            message.capabilities.append(capability)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CapabilityProfileNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
