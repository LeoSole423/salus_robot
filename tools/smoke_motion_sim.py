#!/usr/bin/env python3
"""Exercise the isolated simulated vehicle through the migrated control API."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from salus_interfaces.msg import CmdVelFinal
from sensor_msgs.msg import JointState
from smoke_runtime import SmokeRuntime, finite_odometry, has_increasing_stamps


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


def _yaw(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def main() -> int:
    rclpy.init()
    node = MotionSmokeNode()
    report_path = Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "motion_probe.json"
    runtime = SmokeRuntime(node, "motion-free-world", report_path)
    success = False
    failure = None
    try:
        runtime.wait_publisher_match("command DDS link", node.publisher)
        runtime.wait_topic_publishers("odom publisher", "/odom_raw")
        runtime.wait_topic_publishers("joint publisher", "/joint_states")
        runtime.wait(
            "progressive motion samples",
            lambda: has_increasing_stamps(node.odom_samples)
            and finite_odometry(node.odom_samples[-1]) and node.joint_states >= 2,
            20.0,
            stimulate=lambda: node.command(linear_x=0.0, angular_z=0.0, brake_pct=100),
            observe=lambda: {"odom": len(node.odom_samples), "joint_states": node.joint_states},
        )

        start = node.odom_samples[-1].pose.pose.position
        runtime.wait(
            "straight displacement",
            lambda: math.hypot(node.odom_samples[-1].pose.pose.position.x - start.x,
                               node.odom_samples[-1].pose.pose.position.y - start.y) > 0.10,
            20.0,
            stimulate=lambda: node.command(linear_x=1.0, angular_z=0.0, brake_pct=0),
        )

        start_yaw = _yaw(node.odom_samples[-1])
        runtime.wait(
            "turn yaw change",
            lambda: abs(_yaw(node.odom_samples[-1]) - start_yaw) > 0.03,
            20.0,
            stimulate=lambda: node.command(linear_x=1.0, angular_z=0.20, brake_pct=0),
        )

        runtime.wait(
            "zero brake actuation",
            lambda: node.latest_actuation is not None
            and abs(node.latest_actuation.linear.x) <= 1.0e-9
            and abs(node.latest_actuation.angular.z) <= 1.0e-9,
            10.0,
            stimulate=lambda: node.command(linear_x=1.0, angular_z=0.20, brake_pct=100),
        )
        print("Motion simulation smoke test passed")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "odom_samples": len(node.odom_samples),
            "joint_states": node.joint_states,
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
