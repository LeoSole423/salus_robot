#!/usr/bin/env python3
"""Check that the obstacle-drag world is visible in the clean scan."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from salus_evaluation.models import Pose2D
from salus_evaluation.static_scan_metrics import (
    interpolate_pose, load_obstacle_geometry, scan_static_error_metrics,
    summarize_scan_metrics,
)
from smoke_runtime import SmokeRuntime


@dataclass(frozen=True)
class TimedScan:
    """A scan retained with the ROS timestamp used for ground-truth pairing."""

    stamp_s: float
    message: LaserScan


def _stamp_s(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


class ObstacleDragProbe(Node):
    """Keep scans and timestamped ground-truth poses while the sim moves."""

    def __init__(self) -> None:
        super().__init__("obstacle_drag_smoke")
        self.scans: list[TimedScan] = []
        self.poses: list[tuple[float, Pose2D]] = []
        self.odom_received = 0
        self.create_subscription(
            LaserScan, "/scan_clean", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, 10)

    def _on_scan(self, message: LaserScan) -> None:
        self.scans.append(TimedScan(_stamp_s(message), message))

    def _on_odom(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        pose = Pose2D(
            message.pose.pose.position.x, message.pose.pose.position.y, yaw
        )
        stamp_s = _stamp_s(message)
        if self.poses and stamp_s <= self.poses[-1][0]:
            return
        self.poses.append((stamp_s, pose))
        self.odom_received += 1

    @property
    def motion_observed(self) -> bool:
        if len(self.poses) < 2:
            return False
        first = self.poses[0][1]
        last = self.poses[-1][1]
        return (
            math.hypot(last.x_m - first.x_m, last.y_m - first.y_m) > 0.2
            and abs(last.yaw_rad - first.yaw_rad) > 0.1
        )


def _source_sha() -> str:
    declared = os.environ.get("SMOKE_SOURCE_SHA", "")
    if declared:
        return declared
    try:
        return subprocess.check_output(
            ["git", "-C", "/ros2_ws", "rev-parse", "HEAD"],
            text=True, stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--report-path", type=Path, required=True)
    parser.add_argument("--metrics-path", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=30.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    fixed_frame, obstacles = load_obstacle_geometry(args.geometry)
    rclpy.init()
    node = ObstacleDragProbe()
    runtime = SmokeRuntime(
        node, "obstacle-drag", args.report_path, global_timeout_s=args.timeout
    )
    success = False
    failure: Exception | None = None
    timed_metrics = []
    outlier_records = []
    try:
        runtime.wait(
            "scan_clean messages", lambda: len(node.scans) >= 5, args.timeout,
            observe=lambda: {
                "scans": len(node.scans), "odom": node.odom_received,
            },
        )
        if node.odom_received == 0:
            raise RuntimeError(
                "/odom_raw did not provide a pose for the fixed-frame transform"
            )
        runtime.wait(
            "bounded turn observed",
            lambda: node.motion_observed and len(node.scans) >= 20,
            args.timeout,
            observe=lambda: {
                "scans": len(node.scans), "odom": node.odom_received,
                "motion_observed": node.motion_observed,
            },
        )
        pose_matched = 0
        pose_unmatched = 0
        for timed_scan in node.scans[-20:]:
            scan = timed_scan.message
            pose = interpolate_pose(node.poses, timed_scan.stamp_s)
            if pose is None:
                pose_unmatched += 1
                continue
            pose_matched += 1
            metrics = scan_static_error_metrics(
                scan.ranges, scan.angle_min, scan.angle_increment, pose, obstacles,
                range_min_m=max(0.0, scan.range_min), range_max_m=scan.range_max,
            )
            if metrics.sample_count:
                timed_metrics.append((timed_scan.stamp_s, metrics))
                outlier_records.append({
                    "scan_stamp_s": timed_scan.stamp_s,
                    "beam_indices": list(metrics.worst_beam_indices),
                    "errors_m": list(metrics.worst_errors_m),
                })
        if len(timed_metrics) < 2:
            raise RuntimeError(
                "/scan_clean did not contain known near/mid/far obstacle hits"
            )
        summary = summarize_scan_metrics(timed_metrics)
        report = {
            "schema_version": 2,
            "source_sha": _source_sha(),
            "world": "obstacle_drag.world",
            "geometry_fixture": str(args.geometry),
            "fixed_frame": fixed_frame,
            "sensor_profile": os.environ.get("SMOKE_SENSOR_PROFILE", "clean"),
            "sensor_seed": int(os.environ.get("SMOKE_SENSOR_SEED", "6400")),
            "scan_topic": "/scan_clean",
            "scan_frame": node.scans[-1].message.header.frame_id,
            "odom_topic": "/odom_raw",
            "scan_count": len(node.scans),
            "odom_count": node.odom_received,
            "pose_matched_scan_count": pose_matched,
            "pose_unmatched_scan_count": pose_unmatched,
            **summary,
            "worst_outliers": sorted(
                outlier_records,
                key=lambda item: max(item["errors_m"]),
                reverse=True,
            )[:5],
        }
        args.report_path.parent.mkdir(parents=True, exist_ok=True)
        args.metrics_path.parent.mkdir(parents=True, exist_ok=True)
        args.metrics_path.write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        print(f"Obstacle-drag scan smoke passed; metrics: {args.metrics_path}")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "scan_count": len(node.scans), "odom_count": node.odom_received,
            "pose_count": len(node.poses),
            "metric_samples": [
                metrics.sample_count for _, metrics in timed_metrics
            ],
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
