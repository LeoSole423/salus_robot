#!/usr/bin/env python3
"""Exercise the isolated collision-monitor and command-arbitration chain."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from salus_interfaces.msg import CmdVelFinal, NavTelemetry
from salus_interfaces.srv import BrakeNav, SetManualMode
from tf2_ros import Buffer, TransformException, TransformListener


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
        self.inputs_sent = {"scan": 0, "cmd_vel": 0, "teleop": 0}
        self.phases: list[dict[str, object]] = []
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan_clean", qos_profile_sensor_data
        )
        self.teleop_pub = self.create_publisher(CmdVelFinal, "/cmd_vel_teleop", 10)
        self.final_sub = self.create_subscription(
            CmdVelFinal, "/cmd_vel_final", self.final.append, 10
        )
        self.create_subscription(Twist, "/cmd_vel_safe", self.safe.append, 10)
        self.create_subscription(
            CollisionMonitorState, "/collision_monitor_state", self.states.append, 10
        )
        self.create_subscription(
            NavTelemetry,
            "/nav_command_server/telemetry",
            self.telemetry.append,
            10,
        )
        self.manual_client = self.create_client(
            SetManualMode, "/nav_command_server/set_manual_mode"
        )
        self.brake_client = self.create_client(BrakeNav, "/nav_command_server/brake")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

    def connections(self) -> dict[str, int]:
        return {
            "cmd_vel_subscribers": self.cmd_pub.get_subscription_count(),
            "scan_clean_subscribers": self.scan_pub.get_subscription_count(),
            "teleop_subscribers": self.teleop_pub.get_subscription_count(),
            "cmd_vel_final_publishers": self.count_publishers("/cmd_vel_final"),
        }

    def has_odom_to_base_transform(self) -> bool:
        try:
            self.tf_buffer.lookup_transform("odom", "base_footprint", Time())
        except TransformException:
            return False
        return True

    def publish_inputs(self, obstacle_range: float | None) -> None:
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
            # Collision-monitor polygons require multiple points.  The compact
            # cluster represents one physical obstacle without relying on the
            # simulated LiDAR or perception pipeline.
            for index in range(176, 185):
                scan.ranges[index] = obstacle_range
        command = Twist()
        command.linear.x = 1.0
        self.scan_pub.publish(scan)
        self.cmd_pub.publish(command)
        self.inputs_sent["scan"] += 1
        self.inputs_sent["cmd_vel"] += 1

    def report(self) -> dict[str, object]:
        def final_message(message: CmdVelFinal) -> dict[str, object]:
            return {
                "linear_x": message.twist.linear.x,
                "angular_z": message.twist.angular.z,
                "brake_pct": message.brake_pct,
                "source": message.source,
            }

        def twist_message(message: Twist) -> dict[str, float]:
            return {"linear_x": message.linear.x, "angular_z": message.angular.z}

        return {
            "connections": self.connections(),
            "odom_to_base_footprint_available": self.has_odom_to_base_transform(),
            "inputs_sent": self.inputs_sent,
            "received": {
                "cmd_vel_final": len(self.final),
                "cmd_vel_safe": len(self.safe),
                "collision_monitor_state": len(self.states),
                "telemetry": len(self.telemetry),
            },
            "last": {
                "cmd_vel_final": final_message(self.final[-1]) if self.final else None,
                "cmd_vel_safe": twist_message(self.safe[-1]) if self.safe else None,
                "collision_monitor_action": self.states[-1].action_type if self.states else None,
                "telemetry": {
                    "manual_enabled": self.telemetry[-1].manual_enabled,
                    "cmd_vel_safe_fresh": self.telemetry[-1].cmd_vel_safe_fresh,
                    "cmd_vel_safe_age_s": self.telemetry[-1].cmd_vel_safe_age_s,
                    "failure_code": self.telemetry[-1].failure_code,
                }
                if self.telemetry
                else None,
            },
            "phases": self.phases,
        }


def wait_for(node: SafetySmokeNode, predicate, timeout_s: float, error: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise RuntimeError(error)


def drive_until(
    node: SafetySmokeNode,
    name: str,
    obstacle_range: float | None,
    predicate,
    timeout_s: float,
    error: str,
) -> None:
    started_at = time.monotonic()
    deadline = started_at + timeout_s
    while time.monotonic() < deadline:
        node.publish_inputs(obstacle_range)
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            node.phases.append(
                {
                    "name": name,
                    "success": True,
                    "duration_s": time.monotonic() - started_at,
                    "inputs_sent": dict(node.inputs_sent),
                }
            )
            return
    node.phases.append(
        {
            "name": name,
            "success": False,
            "duration_s": time.monotonic() - started_at,
            "inputs_sent": dict(node.inputs_sent),
            "error": error,
        }
    )
    raise RuntimeError(error)


def call(node: SafetySmokeNode, client, request, error: str):
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError(error)
    future = client.call_async(request)
    wait_for(node, future.done, 5.0, error)
    return future.result()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--report-path",
        type=Path,
        default=Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "safety_probe.json",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = SafetySmokeNode()
    success = False
    try:
        wait_for(node, lambda: bool(node.telemetry), 20.0, "nav command server did not publish telemetry")
        wait_for(
            node,
            lambda: node.connections()["cmd_vel_subscribers"] >= 1,
            20.0,
            "/cmd_vel publisher did not discover collision_monitor",
        )
        wait_for(
            node,
            lambda: node.connections()["scan_clean_subscribers"] >= 2,
            20.0,
            "/scan_clean publisher did not discover collision_monitor and nav_command_server",
        )
        wait_for(
            node,
            lambda: node.connections()["teleop_subscribers"] >= 1,
            20.0,
            "/cmd_vel_teleop publisher did not discover nav_command_server",
        )
        wait_for(
            node,
            lambda: node.connections()["cmd_vel_final_publishers"] >= 1,
            20.0,
            "/cmd_vel_final subscriber did not discover nav_command_server",
        )
        wait_for(
            node,
            node.has_odom_to_base_transform,
            20.0,
            "odom -> base_footprint TF fixture is unavailable",
        )
        response = call(
            node,
            node.manual_client,
            SetManualMode.Request(enabled=False),
            "manual-mode service unavailable",
        )
        if not response.ok or response.enabled_after:
            raise RuntimeError("automatic mode was not accepted before the safety scenario")
        node.phases.append({"name": "automatic_initial", "success": True})

        node.final.clear()
        node.safe.clear()
        drive_until(
            node,
            "clear_auto",
            None,
            lambda: (
                any(message.linear.x > 0.5 for message in node.safe)
                and any(
                    message.source == CmdVelFinal.SOURCE_AUTO and message.twist.linear.x > 0.5
                    for message in node.final
                )
            ),
            10.0,
            "clear automatic command did not reach /cmd_vel_safe and /cmd_vel_final",
        )

        node.final.clear()
        node.safe.clear()
        drive_until(
            node,
            "obstacle_stop",
            0.5,
            lambda: (
                any(message.linear.x == 0.0 and message.angular.z == 0.0 for message in node.safe)
                and any(
                    message.twist.linear.x == 0.0 and message.twist.angular.z == 0.0
                    for message in node.final
                )
            ),
            10.0,
            "collision stop did not reach /cmd_vel_safe and /cmd_vel_final",
        )

        response = call(
            node,
            node.manual_client,
            SetManualMode.Request(enabled=True),
            "manual-mode service unavailable",
        )
        if not response.ok or not response.enabled_after:
            raise RuntimeError("manual mode was rejected")
        node.phases.append({"name": "manual_enabled", "success": True})
        manual = CmdVelFinal()
        manual.twist.linear.x = 0.4
        manual.source = CmdVelFinal.SOURCE_MANUAL
        node.final.clear()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            node.teleop_pub.publish(manual)
            node.inputs_sent["teleop"] += 1
            rclpy.spin_once(node, timeout_sec=0.05)
            if any(
                message.source == CmdVelFinal.SOURCE_MANUAL and message.twist.linear.x == 0.4
                for message in node.final
            ):
                break
        else:
            raise RuntimeError("manual command was not arbitrated")
        node.phases.append({"name": "manual_command", "success": True})
        wait_for(
            node,
            lambda: any(
                message.source == CmdVelFinal.SOURCE_MANUAL and message.twist.linear.x == 0.0
                for message in node.final
            ),
            2.0,
            "manual watchdog did not stop the command",
        )
        node.phases.append({"name": "manual_watchdog", "success": True})

        node.final.clear()
        response = call(
            node,
            node.brake_client,
            BrakeNav.Request(duration_s=0.3, brake_pct=100),
            "brake service unavailable",
        )
        if not response.ok:
            raise RuntimeError(response.error)
        wait_for(
            node,
            lambda: any(
                message.source == CmdVelFinal.SOURCE_SAFETY and message.brake_pct == 100
                for message in node.final
            ),
            2.0,
            "brake hold did not publish an E-stop command",
        )
        node.phases.append({"name": "brake_hold", "success": True})
        response = call(
            node,
            node.manual_client,
            SetManualMode.Request(enabled=False),
            "manual-mode service unavailable",
        )
        if not response.ok or response.enabled_after:
            raise RuntimeError("automatic mode was not restored after the safety smoke")
        wait_for(
            node,
            lambda: bool(node.telemetry) and not node.telemetry[-1].manual_enabled,
            2.0,
            "telemetry did not confirm automatic mode after the safety smoke",
        )
        node.phases.append({"name": "automatic_restored", "success": True})
        success = True
        print("Safety arbitration simulation smoke test passed")
        return 0
    finally:
        report = node.report() | {"success": success}
        args.report_path.parent.mkdir(parents=True, exist_ok=True)
        args.report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
