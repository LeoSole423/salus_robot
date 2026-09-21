#!/usr/bin/env python3
"""Observe costmap persistence across one and two repeated turns."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from salus_evaluation.costmap_drag_metrics import (
    CostmapObservation, CostmapSnapshot, occupied_grid_points,
    summarize_costmap_observations, transform_costmap_points_to_odom,
    transform_odom_points_to_base,
)
from salus_evaluation.models import Pose2D
from salus_evaluation.static_scan_metrics import (
    interpolate_pose, load_obstacle_geometry,
)


@dataclass(frozen=True)
class TimedGrid:
    snapshot: CostmapSnapshot
    received_monotonic_ns: int


def _stamp_s(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _yaw_from_quaternion(rotation) -> float:
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
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


class CostmapDragProbe(Node):
    """Retain costmaps, scans, phases and poses without publishing anything."""

    def __init__(self, repetitions: int) -> None:
        super().__init__("obstacle_drag_costmap_probe")
        self.repetitions = repetitions
        self.local_grids: list[TimedGrid] = []
        self.global_grids: list[TimedGrid] = []
        self.scans: list[tuple[float, tuple[tuple[float, float], ...]]] = []
        self.poses: list[tuple[float, Pose2D]] = []
        self.phases: list[tuple[float, str]] = []
        self.dropped_observations = {
            "local_transform": 0,
            "local_pose": 0,
            "global_transform": 0,
            "global_pose": 0,
        }
        self.tf_buffer = Buffer(cache_time=Duration(seconds=180.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", self._on_local, 10
        )
        self.create_subscription(
            Costmap, "/global_costmap/costmap_raw", self._on_global, 10
        )
        self.create_subscription(
            LaserScan, "/scan_clean", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, 10)
        self.create_subscription(String, "/obstacle_drag/phase", self._on_phase, 10)

    def _on_odom(self, message: Odometry) -> None:
        stamp_s = _stamp_s(message)
        if self.poses and stamp_s <= self.poses[-1][0]:
            return
        self.poses.append((
            stamp_s,
            Pose2D(
                message.pose.pose.position.x, message.pose.pose.position.y,
                _yaw_from_quaternion(message.pose.pose.orientation),
            ),
        ))

    def _on_phase(self, message: String) -> None:
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        self.phases.append((now_s, message.data))

    def _retain_grid(self, target: list[TimedGrid], message: Costmap) -> None:
        info = message.metadata
        target.append(TimedGrid(CostmapSnapshot(
            stamp_s=_stamp_s(message),
            frame_id=message.header.frame_id,
            resolution_m=float(info.resolution),
            origin_x_m=float(info.origin.position.x),
            origin_y_m=float(info.origin.position.y),
            width=int(info.size_x), height=int(info.size_y),
            data=tuple(int(value) for value in message.data),
            origin_yaw_rad=_yaw_from_quaternion(info.origin.orientation),
        ), time.monotonic_ns()))

    def _on_local(self, message: Costmap) -> None:
        self._retain_grid(self.local_grids, message)

    def _on_global(self, message: Costmap) -> None:
        self._retain_grid(self.global_grids, message)

    def _on_scan(self, message: LaserScan) -> None:
        stamp_s = _stamp_s(message)
        pose = interpolate_pose(self.poses, stamp_s)
        if pose is None:
            return
        points = []
        for index, raw_range in enumerate(message.ranges):
            range_m = float(raw_range)
            if not math.isfinite(range_m) or range_m < message.range_min:
                continue
            if range_m > message.range_max:
                continue
            angle = message.angle_min + index * message.angle_increment
            points.append((
                pose.x_m + math.cos(pose.yaw_rad + angle) * range_m,
                pose.y_m + math.sin(pose.yaw_rad + angle) * range_m,
            ))
        self.scans.append((stamp_s, tuple(points)))

    @property
    def done(self) -> bool:
        names = {name for _, name in self.phases}
        required = {"turn_1"}
        if self.repetitions == 2:
            required.update(("pause_1", "turn_2"))
        return "done" in names and required.issubset(names)

    def _map_to_odom(self, points, frame_id: str, stamp_s: float):
        if frame_id in ("odom", "base_footprint"):
            pose = interpolate_pose(self.poses, stamp_s)
            if pose is None:
                return None
            return transform_costmap_points_to_odom(points, frame_id, pose)
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", frame_id, rclpy.time.Time(seconds=stamp_s)
            ).transform
        except Exception:
            return None
        yaw = _yaw_from_quaternion(transform.rotation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return tuple((
            transform.translation.x + cosine * x - sine * y,
            transform.translation.y + sine * x + cosine * y,
        ) for x, y in points)

    def observations(
        self, grids: list[TimedGrid], label: str,
    ) -> list[CostmapObservation]:
        output = []
        phase_by_stamp = sorted(self.phases)
        for timed in grids:
            points = occupied_grid_points(timed.snapshot)
            odom_points = self._map_to_odom(
                points, timed.snapshot.frame_id, timed.snapshot.stamp_s
            )
            if odom_points is None:
                self.dropped_observations[f"{label}_transform"] += 1
                continue
            phase = "unknown"
            for phase_stamp, name in phase_by_stamp:
                if phase_stamp <= timed.snapshot.stamp_s:
                    phase = name
                else:
                    break
            pose = interpolate_pose(self.poses, timed.snapshot.stamp_s)
            if pose is None:
                self.dropped_observations[f"{label}_pose"] += 1
                continue
            output.append(CostmapObservation(
                stamp_s=timed.snapshot.stamp_s,
                phase=phase,
                odom_points=tuple(odom_points),
                base_points=transform_odom_points_to_base(odom_points, pose),
            ))
        return output


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--geometry", type=Path, required=True)
    parser.add_argument("--metrics-path", type=Path, required=True)
    parser.add_argument("--repetitions", type=int, choices=(1, 2), required=True)
    parser.add_argument("--timeout", type=float, default=90.0)
    args, _ = parser.parse_known_args()
    return args


def main() -> int:
    args = parse_args()
    _, obstacles = load_obstacle_geometry(args.geometry)
    rclpy.init()
    node = CostmapDragProbe(args.repetitions)
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.timeout and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.done:
            raise RuntimeError(
                "did not observe the required maneuver phases and stop"
            )
        required_phases = ("turn_1",) if args.repetitions == 1 else (
            "turn_1", "pause_1", "turn_2"
        )
        local_observations = node.observations(node.local_grids, "local")
        global_observations = node.observations(node.global_grids, "global")
        local = summarize_costmap_observations(
            local_observations, obstacles,
            scan_support=node.scans, required_phases=required_phases,
            cohort_phase="turn_1", require_scan_support=True,
        )
        # The static global map may publish only on map changes, so it cannot
        # be required to contain every maneuver phase.  It still must have
        # valid occupied samples; local rolling-map phases remain the strict
        # repeated-turn gate.
        global_ = summarize_costmap_observations(
            global_observations, obstacles,
            scan_support=node.scans, require_scan_support=True,
        )
        if local["status"] != "measured" or global_["status"] != "measured":
            raise RuntimeError(
                "local/global costmap did not yield measurable occupied cells: "
                f"local={local['status']} valid={local['valid_occupied_observation_count']} "
                f"phases={local['phase_counts']} missing={local['missing_phases']}; "
                f"global={global_['status']} valid={global_['valid_occupied_observation_count']} "
                f"phases={global_['phase_counts']} missing={global_['missing_phases']}"
            )
        common_phases = ("turn_1", "pause_1")
        common_local = summarize_costmap_observations(
            local_observations, obstacles,
            scan_support=node.scans, required_phases=common_phases,
            cohort_phase="turn_1", horizon_s=10.0,
            require_scan_support=True,
        )
        report = {
            "schema_version": 1,
            "source_sha": _source_sha(),
            "world": "obstacle_drag.world",
            "geometry_fixture": str(args.geometry),
            "fixed_frame": "odom",
            "simulation_sensors": {
                "profile": os.environ.get("SMOKE_SENSOR_PROFILE", ""),
                "seed": int(os.environ.get("SMOKE_SENSOR_SEED", "6400")),
            },
            "maneuver": {
                "repetitions": args.repetitions,
                "pause_s": 6.0,
                "costmap_reset": False,
                "required_phases": list(required_phases),
                "phase_samples": len(node.phases),
                "comparison_horizon_s": 10.0,
                "comparison_phases": list(common_phases),
            },
            "topics": {
                "scan": "/scan_clean",
                "local_costmap": "/local_costmap/costmap_raw",
                "global_costmap": "/global_costmap/costmap_raw",
                "odom": "/odom_raw",
            },
            "counts": {
                "local_costmaps": len(node.local_grids),
                "global_costmaps": len(node.global_grids),
                "scan_support": len(node.scans),
                "odom": len(node.poses),
                "dropped_observations": dict(node.dropped_observations),
            },
            "local_costmap": local,
            "local_costmap_common_window": common_local,
            "global_costmap": global_,
        }
        args.metrics_path.parent.mkdir(parents=True, exist_ok=True)
        args.metrics_path.write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        print(f"Obstacle-drag costmap measurement passed; metrics: {args.metrics_path}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
