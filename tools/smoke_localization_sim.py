#!/usr/bin/env python3
"""Verify local EKF output while the simulated vehicle is driven continuously."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from salus_interfaces.msg import CmdVelFinal
from sensor_msgs.msg import Imu


class LocalizationSmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("localization_sim_smoke")
        self.imu_samples = 0
        self.wheel_samples: list[Odometry] = []
        self.local_samples: list[Odometry] = []
        self.publisher = self.create_publisher(CmdVelFinal, "/cmd_vel_final", 10)
        self.create_subscription(Imu, "/imu/data", self._on_imu, 10)
        self.create_subscription(Odometry, "/wheel/odometry", self.wheel_samples.append, 10)
        self.create_subscription(Odometry, "/odometry/local", self.local_samples.append, 10)

    def _on_imu(self, _message: Imu) -> None:
        self.imu_samples += 1

    def command(self, linear_x: float, angular_z: float, brake_pct: int = 0) -> None:
        message = CmdVelFinal()
        message.twist.linear.x = linear_x
        message.twist.angular.z = angular_z
        message.brake_pct = brake_pct
        message.source = CmdVelFinal.SOURCE_AUTO
        self.publisher.publish(message)


def spin_with_command(node: LocalizationSmokeNode, duration_s: float, command: tuple[float, float, int]) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.command(*command)
        rclpy.spin_once(node, timeout_sec=0.05)


def yaw(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))


def main() -> int:
    rclpy.init()
    node = LocalizationSmokeNode()
    try:
        spin_with_command(node, 3.0, (1.0, 0.0, 0))
        if node.imu_samples == 0 or not node.wheel_samples or not node.local_samples:
            raise RuntimeError("localization inputs or /odometry/local were not published")
        start = node.local_samples[-1]
        spin_with_command(node, 2.0, (1.0, 0.20, 0))
        end = node.local_samples[-1]
        if end.pose.pose.position.x <= start.pose.pose.position.x + 0.05:
            raise RuntimeError("local EKF did not advance under a forward command")
        if abs(yaw(end) - yaw(start)) <= 0.02:
            raise RuntimeError("local EKF yaw did not change under a turn command")
        spin_with_command(node, 0.5, (1.0, 0.20, 100))
        if abs(node.wheel_samples[-1].twist.twist.linear.x) > 1.0e-6:
            raise RuntimeError("wheel odometry did not report a stop after brake")
        print("Local localization simulation smoke test passed")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
