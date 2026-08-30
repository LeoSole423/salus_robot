#!/usr/bin/env python3
"""Persistent semantic probe for the integrated simulation smoke test."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from salus_interfaces.srv import EvaluatePathHealth, GetNavState, SetSimBatteryPreset
from salus_interfaces.msg import CmdVelFinal
from smoke_runtime import SmokeRuntime, TopicEvidence, subscribe_navigation_startup
from tf2_ros import Buffer, TransformException, TransformListener


class ValidationError(ValueError):
    """A received ROS message violates the integration contract."""


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _finite(values: list[float] | tuple[float, ...], name: str) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise ValidationError(f"{name} contains NaN or Inf")


def validate_odometry(message: Odometry) -> None:
    if stamp_ns(message) <= 0:
        raise ValidationError("odometry timestamp is zero")
    if message.header.frame_id != "map":
        raise ValidationError(f"odometry frame is {message.header.frame_id!r}, expected 'map'")
    if message.child_frame_id != "base_footprint":
        raise ValidationError(
            f"odometry child frame is {message.child_frame_id!r}, expected 'base_footprint'"
        )
    pose = message.pose.pose
    twist = message.twist.twist
    _finite(
        [
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        ],
        "odometry pose or twist",
    )
    _finite(list(message.pose.covariance), "odometry pose covariance")
    _finite(list(message.twist.covariance), "odometry twist covariance")
    quaternion_norm = math.sqrt(
        pose.orientation.x**2
        + pose.orientation.y**2
        + pose.orientation.z**2
        + pose.orientation.w**2
    )
    if not 0.95 <= quaternion_norm <= 1.05:
        raise ValidationError(f"odometry quaternion norm is {quaternion_norm:.6f}, expected ~1")


def validate_scan(message: LaserScan) -> None:
    if stamp_ns(message) <= 0:
        raise ValidationError("scan timestamp is zero")
    if message.header.frame_id != "base_footprint":
        raise ValidationError(f"scan frame is {message.header.frame_id!r}, expected 'base_footprint'")
    _finite(
        [
            message.angle_min,
            message.angle_max,
            message.angle_increment,
            message.time_increment,
            message.scan_time,
            message.range_min,
            message.range_max,
        ],
        "scan metadata",
    )
    if message.angle_increment <= 0.0 or message.angle_max <= message.angle_min:
        raise ValidationError("scan angular metadata is inconsistent")
    if message.range_min < 0.0 or message.range_max <= message.range_min:
        raise ValidationError("scan range metadata is inconsistent")
    if not message.ranges:
        raise ValidationError("scan has no ranges")
    if message.intensities and len(message.intensities) != len(message.ranges):
        raise ValidationError("scan intensities do not match ranges")
    has_valid_range = False
    has_valid_inf = False
    for value in message.ranges:
        value = float(value)
        if math.isnan(value):
            raise ValidationError("scan contains NaN range")
        if math.isinf(value):
            has_valid_inf = value > 0.0
        elif message.range_min <= value <= message.range_max:
            has_valid_range = True
        else:
            raise ValidationError(f"scan range {value} is outside configured limits")
    if not (has_valid_range or has_valid_inf):
        raise ValidationError("scan contains neither usable ranges nor valid +Inf readings")


class IntegrationProbe(Node):
    OPERATIONAL_NODES = {
        "route_executor",
        "patrol_mission_coordinator",
        "nav_snapshot_server",
        "salus_web_gateway",
        "salus_camera",
    }
    OPERATIONAL_SERVICES = {
        "/zones_manager/get_state",
        "/route_executor/get_route_mission_state",
        "/route_executor/get_patrol_mission_state",
        "/nav_snapshot_server/get_nav_snapshot",
        "/camara/camera_ptz_state",
    }

    def __init__(self, *, operational: bool = False) -> None:
        super().__init__("integration_structure_probe", parameter_overrides=[])
        self.operational = operational
        self.odom = TopicEvidence("odometry")
        self.scan = TopicEvidence("scan")
        self.scan_preview = TopicEvidence("scan_preview")
        self.final_commands = 0
        self.startup = subscribe_navigation_startup(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_result: dict[str, Any] = {"map_to_odom": False, "odom_to_base_footprint": False}
        self.required_services = {
            "robot_state_publisher": self.create_client(
                GetParameters, "/robot_state_publisher/get_parameters"
            ),
            "controller": self.create_client(
                SetSimBatteryPreset, "/sim_battery/set_preset"
            ),
            "navigation": self.create_client(
                GetNavState, "/nav_command_server/get_state"
            ),
            "path_health": self.create_client(
                EvaluatePathHealth, "/path_health/evaluate"
            ),
        }
        self.create_subscription(Odometry, "/odometry/global", self._on_odom, QoSProfile(depth=10))
        self.create_subscription(LaserScan, "/scan_clean", self._on_scan, qos_profile_sensor_data)
        if self.operational:
            self.create_subscription(
                LaserScan, "/scan_preview", self._on_scan_preview, qos_profile_sensor_data
            )
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self._on_final_command, 10)

    def _on_odom(self, message: Odometry) -> None:
        self.odom.record(message, validate_odometry)

    def _on_scan(self, message: LaserScan) -> None:
        self.scan.record(message, validate_scan)

    def _on_scan_preview(self, message: LaserScan) -> None:
        self.scan_preview.record(message, validate_scan)

    def _on_final_command(self, _message: CmdVelFinal) -> None:
        self.final_commands += 1

    def node_counts(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        for name, _namespace in self.get_node_names_and_namespaces():
            counts[name] = counts.get(name, 0) + 1
        return counts

    def operational_graph_ready(self) -> bool:
        if not self.operational:
            return True
        counts = self.node_counts()
        available_services = {name for name, _types in self.get_service_names_and_types()}
        return (
            all(counts.get(name, 0) == 1 for name in self.OPERATIONAL_NODES)
            and self.OPERATIONAL_SERVICES.issubset(available_services)
        )

    def graph_ready(self) -> bool:
        counts = self.node_counts()
        return (
            counts.get("robot_state_publisher", 0) == 1
            and self.count_publishers("/cmd_vel_final") == 1
            and self.count_publishers("/scan_3d_raw") >= 1
            and self.operational_graph_ready()
        )

    def check_tf(self) -> bool:
        try:
            map_to_odom = self.tf_buffer.lookup_transform("map", "odom", Time(), timeout=Duration(seconds=0.1))
            odom_to_base = self.tf_buffer.lookup_transform(
                "odom", "base_footprint", Time(), timeout=Duration(seconds=0.1)
            )
        except TransformException as exc:
            self.tf_result["error"] = str(exc)
            return False
        self.tf_result = {
            "map_to_odom": stamp_ns(map_to_odom) > 0,
            "odom_to_base_footprint": stamp_ns(odom_to_base) > 0,
            "map_to_odom_stamp_ns": stamp_ns(map_to_odom),
            "odom_to_base_footprint_stamp_ns": stamp_ns(odom_to_base),
        }
        return all((self.tf_result["map_to_odom"], self.tf_result["odom_to_base_footprint"]))

    def report(self) -> dict[str, Any]:
        return {
            "odometry": self.odom.snapshot(
                publisher_count=self.count_publishers("/odometry/global")
            ),
            "scan": self.scan.snapshot(
                publisher_count=self.count_publishers("/scan_clean")
            ),
            "scan_preview": self.scan_preview.snapshot(
                publisher_count=(
                    self.count_publishers("/scan_preview")
                    if self.operational else None
                )
            ),
            "tf": self.tf_result,
            "services": {
                name: client.service_is_ready()
                for name, client in self.required_services.items()
            },
            "graph": {
                "ready": self.graph_ready(),
                "cmd_vel_final_publishers": self.count_publishers("/cmd_vel_final"),
                "scan_3d_raw_publishers": self.count_publishers("/scan_3d_raw"),
                "final_commands": self.final_commands,
                "node_counts": self.node_counts(),
                "operational_ready": self.operational_graph_ready(),
            },
            "navigation_startup": self.startup.snapshot(),
        }

    def services_ready(self) -> bool:
        return all(client.service_is_ready() for client in self.required_services.values())


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--operational", action="store_true",
        help="Require the full sim_operational graph and compact scan.",
    )
    parser.add_argument(
        "--report-path",
        type=Path,
        default=Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "integration_probe.json",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = IntegrationProbe(operational=args.operational)
    runtime = SmokeRuntime(node, "integration-structure", args.report_path, global_timeout_s=args.timeout)
    success = False
    failure = None
    try:
        runtime.wait_navigation_startup(node.startup, args.timeout)
        runtime.wait_lifecycle("/bt_navigator", 10.0)
        runtime.wait_lifecycle("/collision_monitor", 10.0)
        runtime.wait(
            "integrated contracts valid",
            lambda: (
                node.odom.state(
                    publisher_count=node.count_publishers("/odometry/global")
                ).value == "READY"
                and node.scan.state(
                    publisher_count=node.count_publishers("/scan_clean")
                ).value == "READY"
                and (
                    not node.operational
                    or node.scan_preview.state(
                        publisher_count=node.count_publishers("/scan_preview")
                    ).value == "READY"
                )
                and node.check_tf() and node.services_ready() and node.graph_ready()
            ),
            20.0,
            observe=node.report,
        )
        print(f"Integrated structural probe passed; report: {args.report_path}")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence=node.report())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
