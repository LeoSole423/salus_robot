"""Publish the explicitly selected capability profile as a typed snapshot."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from salus_interfaces.msg import CapabilityState, SystemCapabilities

from .capability_profile import declarations_for_profile, normalize_profile


class CapabilityProfileNode(Node):
    """Latched declaration; it never changes profiles in response to faults."""

    def __init__(self) -> None:
        super().__init__("capability_profile")
        self.declare_parameter("profile", "obstacle_detection")
        self.declare_parameter("output_topic", "/system/capabilities")
        self._profile = normalize_profile(self.get_parameter("profile").value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            SystemCapabilities, str(self.get_parameter("output_topic").value), qos
        )
        self.create_timer(1.0, self._publish)
        self._publish()

    def _publish(self) -> None:
        message = SystemCapabilities()
        message.header.stamp = self.get_clock().now().to_msg()
        message.profile = self._profile
        for item in declarations_for_profile(
            self._profile,
            ready_state=CapabilityState.STATE_ENABLED_BY_PROFILE,
            disabled_state=CapabilityState.STATE_DISABLED_BY_PROFILE,
        ):
            capability = CapabilityState()
            capability.capability_id = item.capability_id
            capability.state = item.state
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
