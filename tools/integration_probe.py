#!/usr/bin/env python3
"""Persistent semantic probe for the integrated simulation smoke test."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
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


@dataclass
class TopicEvidence:
    received: int = 0
    valid: int = 0
    first_latency_s: float | None = None
    timestamps_ns: list[int] = field(default_factory=list)
    frames: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)

    def record(self, message: Any, validator) -> None:
        self.received += 1
        if self.first_latency_s is None:
            self.first_latency_s = time.monotonic() - _PROBE_STARTED_AT
        try:
            validator(message)
        except ValidationError as exc:
            self.errors.append(str(exc))
            return
        self.valid += 1
        self.timestamps_ns.append(stamp_ns(message))
        self.frames.append(message.header.frame_id)

    @property
    def has_progress(self) -> bool:
        return len(self.timestamps_ns) >= 2 and self.timestamps_ns[-1] > self.timestamps_ns[-2]


_PROBE_STARTED_AT = 0.0


class IntegrationProbe(Node):
    def __init__(self) -> None:
        super().__init__("integration_structure_probe", parameter_overrides=[])
        self.odom = TopicEvidence()
        self.scan = TopicEvidence()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_result: dict[str, Any] = {"map_to_odom": False, "odom_to_base_footprint": False}
        self.create_subscription(Odometry, "/odometry/global", self._on_odom, QoSProfile(depth=10))
        self.create_subscription(LaserScan, "/scan_clean", self._on_scan, qos_profile_sensor_data)

    def _on_odom(self, message: Odometry) -> None:
        self.odom.record(message, validate_odometry)

    def _on_scan(self, message: LaserScan) -> None:
        self.scan.record(message, validate_scan)

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
            "odometry": self.odom.__dict__,
            "scan": self.scan.__dict__,
            "tf": self.tf_result,
        }


def _failure_reason(evidence: TopicEvidence, label: str) -> str:
    if evidence.received == 0:
        return f"{label}: topic absent or QoS incompatible (no messages received)"
    if evidence.valid == 0:
        return f"{label}: invalid messages ({'; '.join(evidence.errors[-3:])})"
    if not evidence.has_progress:
        return f"{label}: timestamps stagnant (valid={evidence.valid}, timestamps={evidence.timestamps_ns[-3:]})"
    return ""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--report-path",
        type=Path,
        default=Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "integration_probe.json",
    )
    return parser.parse_args()


def main() -> int:
    global _PROBE_STARTED_AT
    args = parse_args()
    _PROBE_STARTED_AT = time.monotonic()
    rclpy.init()
    node = IntegrationProbe()
    failure = ""
    try:
        deadline = time.monotonic() + args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            odom_failure = _failure_reason(node.odom, "odometry")
            scan_failure = _failure_reason(node.scan, "scan")
            if not odom_failure and not scan_failure and node.check_tf():
                break
        odom_failure = _failure_reason(node.odom, "odometry")
        scan_failure = _failure_reason(node.scan, "scan")
        if odom_failure or scan_failure:
            failure = "; ".join(filter(None, (odom_failure, scan_failure)))
        elif not node.check_tf():
            failure = f"TF unavailable: {node.tf_result.get('error', node.tf_result)}"
        report = node.report() | {"success": not failure, "failure": failure, "timeout_s": args.timeout}
        args.report_path.parent.mkdir(parents=True, exist_ok=True)
        args.report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if failure:
            raise RuntimeError(failure)
        print(f"Integrated structural probe passed; report: {args.report_path}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
