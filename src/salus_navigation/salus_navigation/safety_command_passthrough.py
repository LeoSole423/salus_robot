"""Explicit no-obstacle-detection relay preserving the command boundary."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class SafetyCommandPassthrough(Node):
    """Relay commands without claiming collision protection."""

    def __init__(self) -> None:
        super().__init__("safety_command_passthrough")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")
        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value), self._publisher.publish, 10
        )
        self.get_logger().warning(
            "local obstacle detection is disabled by explicit profile; "
            "commands are relayed without collision monitoring"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafetyCommandPassthrough()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
