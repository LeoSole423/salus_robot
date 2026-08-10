#!/usr/bin/env python3
"""Exercise the isolated simulated vehicle through the migrated control API."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from salus_interfaces.msg import CmdVelFinal
from sensor_msgs.msg import JointState


class MotionSmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("motion_sim_smoke")
        self.odom_samples: list[Odometry] = []
        self.joint_states = 0
        self.latest_actuation: Twist | None = None
        self.publisher = self.create_publisher(CmdVelFinal, "/cmd_vel_final", 10)
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, 10)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(Twist, "/cmd_vel_gazebo", self._on_actuation, 10)

    def _on_odom(self, message: Odometry) -> None:
        self.odom_samples.append(message)

    def _on_joint_state(self, _message: JointState) -> None:
        self.joint_states += 1

    def _on_actuation(self, message: Twist) -> None:
        self.latest_actuation = message

    def command(self, *, linear_x: float, angular_z: float, brake_pct: int) -> None:
        message = CmdVelFinal()
        message.twist.linear.x = linear_x
        message.twist.angular.z = angular_z
        message.brake_pct = brake_pct
        message.source = CmdVelFinal.SOURCE_AUTO
        self.publisher.publish(message)


def _spin_for(node: MotionSmokeNode, duration_s: float, command: tuple[float, float, int]) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.command(
            linear_x=command[0], angular_z=command[1], brake_pct=command[2]
        )
        rclpy.spin_once(node, timeout_sec=0.05)


def _yaw(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def main() -> int:
    rclpy.init()
    node = MotionSmokeNode()
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.odom_samples and node.joint_states > 0:
                break
        if not node.odom_samples or node.joint_states == 0:
            raise RuntimeError("simulation did not publish odometry and joint states")

        start_x = node.odom_samples[-1].pose.pose.position.x
        _spin_for(node, 2.0, (1.0, 0.0, 0))
        straight_end = node.odom_samples[-1]
        if straight_end.pose.pose.position.x <= start_x + 0.10:
            raise RuntimeError("straight command did not produce positive displacement")

        start_yaw = _yaw(straight_end)
        _spin_for(node, 2.0, (1.0, 0.20, 0))
        turn_end = node.odom_samples[-1]
        if abs(_yaw(turn_end) - start_yaw) <= 0.03:
            raise RuntimeError("turn command did not change simulated yaw")

        _spin_for(node, 0.5, (1.0, 0.20, 100))
        if node.latest_actuation is None:
            raise RuntimeError("controller did not publish simulated actuation")
        if abs(node.latest_actuation.linear.x) > 1.0e-9 or abs(node.latest_actuation.angular.z) > 1.0e-9:
            raise RuntimeError("brake did not produce zero simulated actuation")
        print("Motion simulation smoke test passed")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
