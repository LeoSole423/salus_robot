#!/usr/bin/env python3
"""Assert that the simulated 3D LiDAR reaches every ROS perception stage."""

from __future__ import annotations

import math
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from smoke_runtime import SmokeRuntime, has_increasing_stamps


class LidarSmokeNode(Node):
    def __init__(self) -> None:
        super().__init__("lidar_sim_smoke")
        self.raw: list[PointCloud2] = []
        self.normalized: list[PointCloud2] = []
        self.obstacles: list[PointCloud2] = []
        self.clean: list[LaserScan] = []
        self.create_subscription(PointCloud2, "/scan_3d_raw", self.raw.append, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/scan_3d", self.normalized.append, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/obstacles_cloud", self.obstacles.append, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan_clean", self.clean.append, qos_profile_sensor_data)


def _has_point_in_front(message: PointCloud2) -> bool:
    for point in point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = map(float, point)
        if 3.0 <= x <= 5.0 and 1.4 <= abs(y) <= 2.6 and -0.2 <= z <= 1.5:
            return True
    return False


def _has_obstacle_range(message: LaserScan) -> bool:
    return any(math.isfinite(value) and 3.0 <= value <= 5.0 for value in message.ranges)


def main() -> int:
    rclpy.init()
    node = LidarSmokeNode()
    runtime = SmokeRuntime(
        node, "lidar-obstacle",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "lidar_probe.json",
    )
    success = False
    failure = None
    try:
        for topic in ("/scan_3d_raw", "/scan_3d", "/obstacles_cloud", "/scan_clean"):
            runtime.wait_topic_publishers(f"publisher {topic}", topic, timeout_s=30.0)
        runtime.wait(
            "continuous lidar chain",
            lambda: has_increasing_stamps(node.raw, 4)
            and has_increasing_stamps(node.normalized)
            and has_increasing_stamps(node.obstacles)
            and has_increasing_stamps(node.clean),
            35.0,
            observe=lambda: {"raw": len(node.raw), "normalized": len(node.normalized),
                             "obstacles": len(node.obstacles), "clean": len(node.clean)},
        )
        raw = node.raw[-1]
        if not raw.header.frame_id or raw.width * raw.height == 0:
            raise RuntimeError("raw LiDAR cloud has an invalid frame or contains no points")
        if raw.header.stamp.sec == 0 and raw.header.stamp.nanosec == 0:
            raise RuntimeError("raw LiDAR cloud has no timestamp")
        if len(node.raw) < 4:
            raise RuntimeError("raw LiDAR cloud is not publishing continuously")
        if node.normalized[-1].header.frame_id != "lidar_link":
            raise RuntimeError("normalized LiDAR cloud does not use lidar_link")
        if not _has_point_in_front(raw):
            raise RuntimeError("raw LiDAR cloud does not contain the collision obstacle")
        if not _has_point_in_front(node.obstacles[-1]):
            raise RuntimeError("ground filter removed the collision obstacle")
        if not _has_obstacle_range(node.clean[-1]):
            raise RuntimeError("clean scan does not retain the collision obstacle")
        print("LiDAR simulation smoke test passed")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "raw": len(node.raw), "normalized": len(node.normalized),
            "obstacles": len(node.obstacles), "clean": len(node.clean),
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
