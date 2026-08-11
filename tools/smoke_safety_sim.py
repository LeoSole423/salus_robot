#!/usr/bin/env python3
"""Exercise the active collision-monitor and command-arbitration chain."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from salus_interfaces.msg import CmdVelFinal, NavTelemetry
from salus_interfaces.srv import BrakeNav, SetManualMode


class SafetySmokeNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "safety_sim_smoke",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.final: list[CmdVelFinal] = []
        self.safe: list[Twist] = []
        self.states: list[CollisionMonitorState] = []
        self.telemetry: list[NavTelemetry] = []
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        # Match the sensor QoS consumed by collision_monitor.  This is a second
        # publisher used only by the smoke test; the normal perception pipeline
        # continues publishing its own scan on the same ROS contract.
        self.scan_pub = self.create_publisher(LaserScan, "/scan_clean", qos_profile_sensor_data)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self.final.append, 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self.safe.append, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self.states.append, 10)
        self.create_subscription(NavTelemetry, "/nav_command_server/telemetry", self.telemetry.append, 10)
        self.manual_client = self.create_client(SetManualMode, "/nav_command_server/set_manual_mode")
        self.brake_client = self.create_client(BrakeNav, "/nav_command_server/brake")

    def publish_scan_and_command(self, obstacle_range: float | None, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = "base_footprint"
            scan.angle_min = -math.pi / 2.0
            scan.angle_max = math.pi / 2.0
            scan.angle_increment = math.pi / 359.0
            scan.range_min = 0.4
            scan.range_max = 20.0
            scan.ranges = [float("inf")] * 360
            if obstacle_range is not None:
                # The legacy polygons require at least three points.  A small
                # cluster represents a physical obstacle much better than one
                # isolated laser ray and exercises that configured threshold.
                for index in range(176, 185):
                    scan.ranges[index] = obstacle_range
            command = Twist()
            command.linear.x = 1.0
            self.scan_pub.publish(scan)
            self.cmd_pub.publish(command)
            rclpy.spin_once(self, timeout_sec=0.05)


def wait_for(node: SafetySmokeNode, predicate, timeout_s: float, error: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise RuntimeError(error)


def call(node: SafetySmokeNode, client, request, error: str):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(error)
    future = client.call_async(request)
    wait_for(node, future.done, 5.0, error)
    return future.result()


def main() -> int:
    rclpy.init()
    node = SafetySmokeNode()
    try:
        wait_for(node, lambda: node.telemetry, 20.0, "nav command server did not publish telemetry")
        node.publish_scan_and_command(None, 2.0)
        wait_for(
            node,
            lambda: any(message.source == CmdVelFinal.SOURCE_AUTO and message.twist.linear.x > 0.5 for message in node.final),
            5.0,
            "clear automatic command did not reach /cmd_vel_final",
        )

        node.final.clear()
        node.safe.clear()
        node.publish_scan_and_command(0.5, 2.0)
        wait_for(
            node,
            lambda: (
                any(message.linear.x == 0.0 and message.angular.z == 0.0 for message in node.safe)
                and any(message.twist.linear.x == 0.0 and message.twist.angular.z == 0.0 for message in node.final)
            ),
            5.0,
            "collision stop did not reach /cmd_vel_final",
        )

        response = call(node, node.manual_client, SetManualMode.Request(enabled=True), "manual-mode service unavailable")
        if not response.ok or not response.enabled_after:
            raise RuntimeError("manual mode was rejected")
        manual = CmdVelFinal()
        manual.twist.linear.x = 0.4
        manual.source = CmdVelFinal.SOURCE_MANUAL
        teleop_pub = node.create_publisher(CmdVelFinal, "/cmd_vel_teleop", 10)
        node.final.clear()
        teleop_pub.publish(manual)
        wait_for(node, lambda: any(message.source == CmdVelFinal.SOURCE_MANUAL and message.twist.linear.x == 0.4 for message in node.final), 3.0, "manual command was not arbitrated")
        wait_for(node, lambda: any(message.source == CmdVelFinal.SOURCE_MANUAL and message.twist.linear.x == 0.0 for message in node.final), 2.0, "manual watchdog did not stop the command")

        node.final.clear()
        response = call(node, node.brake_client, BrakeNav.Request(duration_s=0.3, brake_pct=100), "brake service unavailable")
        if not response.ok:
            raise RuntimeError(response.error)
        wait_for(node, lambda: any(message.source == CmdVelFinal.SOURCE_SAFETY and message.brake_pct == 100 for message in node.final), 2.0, "brake hold did not publish an E-stop command")
        print("Safety arbitration simulation smoke test passed")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
