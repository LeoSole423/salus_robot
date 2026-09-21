#!/usr/bin/env python3
"""Run one bounded open-loop Ackermann maneuver for obstacle-drag measurement."""

from __future__ import annotations

import sys

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


ENTRY_S = 2.0
TURN_S = 10.0
EXIT_S = 2.0
STOP_S = 1.0
SPEED_MPS = 0.5
TURN_RATE_RPS = -0.125  # 4 m radius at 0.5 m/s; right turn stays clear of near post.


class ObstacleDragManeuver(Node):
    """Publish the existing /cmd_vel input through one deterministic sequence."""

    def __init__(self) -> None:
        super().__init__("obstacle_drag_maneuver")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.started_s: float | None = None
        self.timer = self.create_timer(0.04, self._tick)

    def _tick(self) -> None:
        if self.publisher.get_subscription_count() < 1:
            return
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        if self.started_s is None:
            self.started_s = now_s
        elapsed_s = now_s - self.started_s
        message = Twist()
        if elapsed_s < ENTRY_S:
            message.linear.x = SPEED_MPS
        elif elapsed_s < ENTRY_S + TURN_S:
            message.linear.x = SPEED_MPS
            message.angular.z = TURN_RATE_RPS
        elif elapsed_s < ENTRY_S + TURN_S + EXIT_S:
            message.linear.x = SPEED_MPS
        elif elapsed_s < ENTRY_S + TURN_S + EXIT_S + STOP_S:
            pass
        else:
            self.publisher.publish(message)
            self.timer.cancel()
            rclpy.shutdown()
            return
        self.publisher.publish(message)


def main() -> int:
    rclpy.init()
    node = ObstacleDragManeuver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
