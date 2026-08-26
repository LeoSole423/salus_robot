#!/usr/bin/env python3
"""Verify local EKF output while the simulated vehicle is driven continuously."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from salus_interfaces.msg import CmdVelFinal
from sensor_msgs.msg import Imu
from smoke_runtime import SmokeRuntime, finite_odometry, has_increasing_stamps


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


def yaw(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y), 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))


def main() -> int:
    rclpy.init()
    node = LocalizationSmokeNode()
    report_path = Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "localization_probe.json"
    runtime = SmokeRuntime(node, "localization-free-world", report_path)
    success = False
    failure = None
    try:
        runtime.wait_publisher_match("command DDS link", node.publisher)
        for topic in ("/imu/data", "/wheel/odometry", "/odometry/local"):
            runtime.wait_topic_publishers(f"publisher {topic}", topic)
        runtime.wait(
            "single wheel odometry authority",
            lambda: len(node.get_publishers_info_by_topic("/wheel/odometry")) == 1,
            10.0,
            observe=lambda: {
                "publishers": [
                    info.node_name
                    for info in node.get_publishers_info_by_topic("/wheel/odometry")
                ]
            },
        )
        runtime.wait(
            "progressive localization samples",
            lambda: node.imu_samples >= 2
            and has_increasing_stamps(node.wheel_samples)
            and has_increasing_stamps(node.local_samples)
            and finite_odometry(node.wheel_samples[-1])
            and finite_odometry(node.local_samples[-1]),
            25.0,
            stimulate=lambda: node.command(0.0, 0.0, 100),
            observe=lambda: {
                "imu": node.imu_samples,
                "wheel": len(node.wheel_samples),
                "local": len(node.local_samples),
            },
        )
        start = node.local_samples[-1]
        start_xy = (start.pose.pose.position.x, start.pose.pose.position.y)
        runtime.wait(
            "euclidean forward displacement",
            lambda: math.hypot(node.local_samples[-1].pose.pose.position.x - start_xy[0],
                               node.local_samples[-1].pose.pose.position.y - start_xy[1]) > 0.10,
            20.0,
            stimulate=lambda: node.command(1.0, 0.0, 0),
            observe=lambda: {"samples": len(node.local_samples)},
        )
        start_yaw = yaw(node.local_samples[-1])
        runtime.wait(
            "local yaw change",
            lambda: abs(yaw(node.local_samples[-1]) - start_yaw) > 0.03,
            20.0,
            stimulate=lambda: node.command(1.0, 0.20, 0),
            observe=lambda: {"yaw_delta": abs(yaw(node.local_samples[-1]) - start_yaw)},
        )
        runtime.wait(
            "wheel odometry stopped",
            lambda: abs(node.wheel_samples[-1].twist.twist.linear.x) <= 1.0e-6,
            10.0,
            stimulate=lambda: node.command(1.0, 0.20, 100),
            observe=lambda: {"wheel_linear_x": node.wheel_samples[-1].twist.twist.linear.x},
        )
        print("Local localization simulation smoke test passed")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "imu_samples": node.imu_samples,
            "wheel_samples": len(node.wheel_samples),
            "local_samples": len(node.local_samples),
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
