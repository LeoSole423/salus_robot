#!/usr/bin/env python3
"""Check that the obstacle-drag world is visible in the clean scan."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from salus_evaluation.models import Pose2D
from salus_evaluation.stage_metrics import (
    beam_support_is_identical, exact_common_stamps, pointcloud_payload_signature,
    compare_scan_projection, project_pointcloud_to_scan, summarize_point_geometry,
    transform_points_to_odom, validate_stage_lineage,
)
from salus_evaluation.static_scan_metrics import (
    interpolate_pose, load_obstacle_geometry, scan_static_error_metrics,
    summarize_pose_divergence, summarize_scan_metrics,
    summarize_temporal_offset_sweep,
)
from smoke_runtime import SmokeRuntime


@dataclass(frozen=True)
class TimedScan:
    """A scan retained with the ROS timestamp used for ground-truth pairing."""

    stamp_s: float
    message: LaserScan
    receive_monotonic_ns: int = 0


@dataclass(frozen=True)
class TimedCloud:
    """Point cloud retained with source and reception timestamps."""

    stamp_ns: int
    message: PointCloud2
    receive_monotonic_ns: int


@dataclass(frozen=True)
class TimedLocalPose:
    """EKF pose retained for comparison against simulation odometry."""

    stamp_s: float
    pose: Pose2D


@dataclass(frozen=True)
class TimedTfPose:
    """Dynamic odom-to-base pose retained from the authoritative TF stream."""

    stamp_s: float
    pose: Pose2D


def _stamp_s(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ObstacleDragProbe(Node):
    """Keep scans and timestamped ground-truth poses while the sim moves."""

    def __init__(self) -> None:
        super().__init__("obstacle_drag_smoke")
        self.scans: list[TimedScan] = []
        self.scan_input: list[TimedScan] = []
        self.raw_clouds: list[TimedCloud] = []
        self.normalized_clouds: list[TimedCloud] = []
        self.obstacle_clouds: list[TimedCloud] = []
        self.poses: list[tuple[float, Pose2D]] = []
        self.local_poses: list[TimedLocalPose] = []
        self.tf_poses: list[TimedTfPose] = []
        self.odom_received = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(
            PointCloud2, "/scan_3d_raw", self._on_raw_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, "/scan_3d", self._on_normalized_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PointCloud2, "/obstacles_cloud", self._on_obstacle_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan, "/scan", self._on_scan_input, qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan, "/scan_clean", self._on_scan, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom_raw", self._on_odom, 10)
        self.create_subscription(Odometry, "/odometry/local", self._on_local_odom, 10)
        tf_qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(TFMessage, "/tf", self._on_tf, tf_qos)

    def _retain_cloud(self, target: list[TimedCloud], message: PointCloud2) -> None:
        target.append(TimedCloud(_stamp_ns(message), message, time.monotonic_ns()))

    def _on_raw_cloud(self, message: PointCloud2) -> None:
        self._retain_cloud(self.raw_clouds, message)

    def _on_normalized_cloud(self, message: PointCloud2) -> None:
        self._retain_cloud(self.normalized_clouds, message)

    def _on_obstacle_cloud(self, message: PointCloud2) -> None:
        self._retain_cloud(self.obstacle_clouds, message)

    def _on_scan_input(self, message: LaserScan) -> None:
        self.scan_input.append(
            TimedScan(_stamp_s(message), message, time.monotonic_ns())
        )

    def _on_scan(self, message: LaserScan) -> None:
        self.scans.append(TimedScan(_stamp_s(message), message, time.monotonic_ns()))

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

    def _on_local_odom(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        stamp_s = _stamp_s(message)
        if self.local_poses and stamp_s <= self.local_poses[-1].stamp_s:
            return
        self.local_poses.append(TimedLocalPose(
            stamp_s,
            Pose2D(message.pose.pose.position.x, message.pose.pose.position.y, yaw),
        ))

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if (
                transform.header.frame_id != "odom"
                or transform.child_frame_id != "base_footprint"
            ):
                continue
            rotation = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            stamp_s = _stamp_s(transform)
            if self.tf_poses and stamp_s <= self.tf_poses[-1].stamp_s:
                continue
            self.tf_poses.append(TimedTfPose(
                stamp_s,
                Pose2D(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    yaw,
                ),
            ))

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


def _pose_divergence(
    local_poses: list[TimedLocalPose | TimedTfPose],
    raw_poses: list[tuple[float, Pose2D]],
    *,
    pose_count_key: str = "local_pose_count",
) -> dict[str, object]:
    """Summarize a timestamped pose stream versus raw simulation odometry."""
    summary = summarize_pose_divergence(
        [(sample.stamp_s, sample.pose) for sample in local_poses], raw_poses,
    )
    summary[pose_count_key] = summary.pop("pose_count")
    return summary


def _latest_by_stamp(samples):
    """Index retained messages by exact ROS source stamp."""
    indexed = {}
    for sample in samples:
        stamp_ns = (
            sample.stamp_ns
            if isinstance(sample, TimedCloud)
            else _stamp_ns(sample.message)
        )
        indexed[stamp_ns] = sample
    return indexed


def _complete_stage_stamps(node: ObstacleDragProbe) -> set[int]:
    """Return exact stamps present in every perception stage."""
    return set(exact_common_stamps([
        _latest_by_stamp(samples)
        for samples in (
            node.raw_clouds, node.normalized_clouds, node.obstacle_clouds,
            node.scan_input, node.scans,
        )
    ]))


def _cloud_content_signature(message: PointCloud2) -> str:
    """Hash PointCloud2 content while deliberately excluding its frame header."""
    return pointcloud_payload_signature(message)


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low, high = math.floor(index), math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def _quaternion_point(point, transform) -> tuple[float, float, float]:
    """Apply a geometry transform to one XYZ point."""
    x, y, z = point
    q = transform.rotation
    t = transform.translation
    ix = q.w * x + q.y * z - q.z * y
    iy = q.w * y + q.z * x - q.x * z
    iz = q.w * z + q.x * y - q.y * x
    iw = -q.x * x - q.y * y - q.z * z
    return (
        ix * q.w + iw * -q.x + iy * -q.z - iz * -q.y + t.x,
        iy * q.w + iw * -q.y + iz * -q.x - ix * -q.z + t.y,
        iz * q.w + iw * -q.z + ix * -q.x - iy * -q.y + t.z,
    )


def _cloud_points_in_base(node: ObstacleDragProbe, message: PointCloud2):
    """Decode a cloud and express it in base_footprint when its TF is known."""
    points = point_cloud2.read_points(
        message, field_names=("x", "y", "z"), skip_nans=True
    )
    decoded = [(float(row[0]), float(row[1]), float(row[2])) for row in points]
    if message.header.frame_id == "base_footprint":
        return decoded
    if message.header.frame_id != "lidar_link":
        return None
    try:
        transform = node.tf_buffer.lookup_transform(
            "base_footprint", "lidar_link", rclpy.time.Time()
        ).transform
    except Exception:
        return None
    return [_quaternion_point(point, transform) for point in decoded]


def _stage_latency(raw_by_stamp, stage_by_stamp, stamps):
    delays = []
    for stamp_ns in stamps:
        raw = raw_by_stamp.get(stamp_ns)
        stage = stage_by_stamp.get(stamp_ns)
        if raw is None or stage is None:
            continue
        if raw.receive_monotonic_ns and stage.receive_monotonic_ns:
            delays.append(
                (stage.receive_monotonic_ns - raw.receive_monotonic_ns) / 1.0e9
            )
    return {
        "status": "measured" if delays else "insufficient_data",
        "paired_count": len(delays),
        "median_delay_s": (
            _percentile(delays, 0.5) if delays else None
        ),
        "p95_delay_s": _percentile(delays, 0.95),
        "max_delay_s": max(delays) if delays else None,
    }


def _scan_metadata_signature(message: LaserScan) -> tuple[object, ...]:
    """Return the scan contract fields that must survive cleaning."""
    return (
        message.header.frame_id,
        message.angle_min,
        message.angle_max,
        message.angle_increment,
        message.time_increment,
        message.scan_time,
        message.range_min,
        message.range_max,
    )


def _edge_losses(stage_samples: dict[str, list]) -> dict[str, dict[str, int]]:
    """Report exact-stamp loss between each adjacent perception stage."""
    indexed = {
        topic: set(_latest_by_stamp(samples))
        for topic, samples in stage_samples.items()
    }
    topics = list(stage_samples)
    losses = {}
    for source, target in zip(topics, topics[1:]):
        source_stamps, target_stamps = indexed[source], indexed[target]
        losses[f"{source} -> {target}"] = {
            "source_unique_stamp_count": len(source_stamps),
            "target_unique_stamp_count": len(target_stamps),
            "matched_exact_stamp_count": len(source_stamps & target_stamps),
            "source_only_stamp_count": len(source_stamps - target_stamps),
            "target_only_stamp_count": len(target_stamps - source_stamps),
        }
    return losses


def _stage_point_summary(node, samples, stamps, obstacles, raw_poses):
    all_points = []
    transform_missing = 0
    source_point_counts = []
    for stamp_ns in stamps:
        sample = samples.get(stamp_ns)
        pose = interpolate_pose(raw_poses, stamp_ns / 1.0e9)
        if sample is None or pose is None:
            transform_missing += 1
            continue
        base_points = _cloud_points_in_base(node, sample.message)
        if base_points is None:
            transform_missing += 1
            continue
        selected = [
            (x_m, y_m) for x_m, y_m, z_m in base_points
            if 0.10 <= z_m <= 1.60
        ]
        source_point_counts.append(len(selected))
        all_points.extend(transform_points_to_odom(selected, pose))
    geometry = summarize_point_geometry(all_points, obstacles)
    geometry["transform_missing_count"] = transform_missing
    geometry["source_point_count_median"] = _percentile(
        [float(count) for count in source_point_counts], 0.5
    )
    return geometry


def _stage_scan_summary(samples, stamps, raw_poses, obstacles, common_support):
    metrics = []
    for stamp_ns in stamps:
        sample = samples.get(stamp_ns)
        pose = interpolate_pose(raw_poses, stamp_ns / 1.0e9)
        support = common_support.get(stamp_ns, ())
        if sample is None or pose is None or not support:
            continue
        metric = scan_static_error_metrics(
            sample.message.ranges, sample.message.angle_min,
            sample.message.angle_increment, pose, obstacles,
            range_min_m=max(0.0, sample.message.range_min),
            range_max_m=sample.message.range_max,
            beam_indices=support,
        )
        if metric.sample_count:
            metrics.append((stamp_ns / 1.0e9, metric))
    if not metrics:
        return {"status": "insufficient_data", "paired_count": 0}
    return {
        "status": "measured",
        "paired_count": len(metrics),
        **summarize_scan_metrics(metrics),
    }


def _metric_record(metric):
    """Serialize a static geometry metric for a shared beam population."""
    return {
        "sample_count": metric.sample_count,
        "rmse_m": metric.scan_static_error_rmse_m,
        "p95_m": metric.scan_static_error_p95_m,
        "max_m": metric.max_error_m,
        "beam_indices": list(metric.scored_beam_indices),
    }


def _projection_oracle(node, stamps, obstacles):
    """Compare the upstream projection semantics with each runtime /scan."""
    obstacle_clouds = _latest_by_stamp(node.obstacle_clouds)
    scans = _latest_by_stamp(node.scan_input)
    records = []
    all_deltas = []
    all_worst = []
    finite_infinite_mismatches = 0
    invalid_actual_count = 0
    common_finite_support_count = 0
    geometry_paired_count = 0
    for stamp_ns in stamps:
        cloud = obstacle_clouds.get(stamp_ns)
        scan = scans.get(stamp_ns)
        if cloud is None or scan is None:
            continue
        points = _cloud_points_in_base(node, cloud.message)
        if points is None:
            continue
        message = scan.message
        oracle_ranges = project_pointcloud_to_scan(
            points,
            angle_min=message.angle_min,
            angle_max=message.angle_max,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
            min_height=-0.1,
            max_height=1.6,
            use_inf=True,
            inf_epsilon=1.0,
        )
        comparison = compare_scan_projection(oracle_ranges, message.ranges)
        pose = interpolate_pose(node.poses, stamp_ns / 1.0e9)
        common_support = comparison["common_finite_support"]
        common_finite_support_count += len(common_support)
        oracle_geometry = None
        actual_geometry = None
        if pose is not None and common_support:
            oracle_geometry = _metric_record(scan_static_error_metrics(
                oracle_ranges, message.angle_min, message.angle_increment, pose,
                obstacles, range_min_m=max(0.0, message.range_min),
                range_max_m=message.range_max, beam_indices=common_support,
            ))
            actual_geometry = _metric_record(scan_static_error_metrics(
                message.ranges, message.angle_min, message.angle_increment, pose,
                obstacles, range_min_m=max(0.0, message.range_min),
                range_max_m=message.range_max, beam_indices=common_support,
            ))
            if oracle_geometry["sample_count"] and actual_geometry["sample_count"]:
                geometry_paired_count += 1
        for index, (expected, actual) in enumerate(
            zip(oracle_ranges, message.ranges)
        ):
            if math.isfinite(float(expected)) and math.isfinite(float(actual)):
                all_deltas.append(abs(float(expected) - float(actual)))
                all_worst.append({
                    "stamp_ns": stamp_ns,
                    "beam_index": index,
                    "abs_delta_m": abs(float(expected) - float(actual)),
                })
        finite_infinite_mismatches += comparison[
            "finite_infinite_mismatch_count"
        ]
        invalid_actual_count += comparison["invalid_actual_count"]
        records.append({
            "stamp_ns": stamp_ns,
            "point_count": len(points),
            "comparison": comparison,
            "common_finite_support_count": len(common_support),
            "oracle_geometry": oracle_geometry,
            "actual_geometry": actual_geometry,
        })
    all_worst.sort(key=lambda item: float(item["abs_delta_m"]), reverse=True)
    return {
        "status": "measured" if records else "insufficient_data",
        "algorithm": {
            "name": "pointcloud_to_laserscan_projection",
            "upstream_repository": "ros-perception/pointcloud_to_laserscan",
            "upstream_ref": "humble",
            "validated_against_version": "2.0.1",
        },
        "parameters": {
            "min_height": -0.1,
            "max_height": 1.6,
            "use_inf": True,
            "inf_epsilon": 1.0,
            "coordinate_frame": "base_footprint",
        },
        "paired_count": len(records),
        "finite_infinite_mismatch_count": finite_infinite_mismatches,
        "invalid_actual_count": invalid_actual_count,
        "common_finite_support_count": common_finite_support_count,
        "geometry_paired_count": geometry_paired_count,
        "range_delta_m": {
            "status": "measured" if all_deltas else "insufficient_data",
            "median": _percentile(all_deltas, 0.50) if all_deltas else None,
            "p95": _percentile(all_deltas, 0.95) if all_deltas else None,
            "max": max(all_deltas) if all_deltas else None,
        },
        "worst_bins": all_worst[:10],
        "per_stamp": records,
    }


def _stage_lineage(node, obstacles, complete_stamps):
    """Build a bounded, report-only record for the complete perception chain."""
    selected_stamps = tuple(sorted(complete_stamps)[-20:])
    raw = _latest_by_stamp(node.raw_clouds)
    normalized = _latest_by_stamp(node.normalized_clouds)
    obstacle_cloud = _latest_by_stamp(node.obstacle_clouds)
    scan_input = _latest_by_stamp(node.scan_input)
    scan_clean = _latest_by_stamp(node.scans)
    raw_support = {}
    clean_support = {}
    scan_support_equal = True
    for stamp_ns in selected_stamps:
        pose = interpolate_pose(node.poses, stamp_ns / 1.0e9)
        if pose is None:
            continue
        input_metric = scan_static_error_metrics(
            scan_input[stamp_ns].message.ranges,
            scan_input[stamp_ns].message.angle_min,
            scan_input[stamp_ns].message.angle_increment,
            pose,
            obstacles,
            range_min_m=max(0.0, scan_input[stamp_ns].message.range_min),
            range_max_m=scan_input[stamp_ns].message.range_max,
        )
        clean_metric = scan_static_error_metrics(
            scan_clean[stamp_ns].message.ranges,
            scan_clean[stamp_ns].message.angle_min,
            scan_clean[stamp_ns].message.angle_increment,
            pose,
            obstacles,
            range_min_m=max(0.0, scan_clean[stamp_ns].message.range_min),
            range_max_m=scan_clean[stamp_ns].message.range_max,
        )
        common = tuple(sorted(
            set(input_metric.scored_beam_indices)
            & set(clean_metric.scored_beam_indices)
        ))
        scan_support_equal &= beam_support_is_identical(
            input_metric.scored_beam_indices, clean_metric.scored_beam_indices
        )
        raw_support[stamp_ns] = common
        clean_support[stamp_ns] = common

    scan_metadata_equal = all(
        _scan_metadata_signature(scan_input[stamp].message)
        == _scan_metadata_signature(scan_clean[stamp].message)
        for stamp in selected_stamps
    )

    def cloud_stage(name, samples, *, geometry_samples=None, geometry_note=None):
        indexed = samples
        frame_ids = sorted({indexed[stamp].message.header.frame_id for stamp in selected_stamps})
        geometry_samples = indexed if geometry_samples is None else geometry_samples
        return {
            "topic": name,
            "matched_count": len(selected_stamps),
            "frame_ids": frame_ids,
            "content_signatures": {
                str(stamp): _cloud_content_signature(indexed[stamp].message)
                for stamp in selected_stamps
            },
            "latency_vs_raw": _stage_latency(raw, indexed, selected_stamps),
            "geometry": _stage_point_summary(
                node, geometry_samples, selected_stamps, obstacles, node.poses
            ),
            **({"geometry_note": geometry_note} if geometry_note else {}),
        }

    raw_signatures = {
        stamp: _cloud_content_signature(raw[stamp].message)
        for stamp in selected_stamps
    }
    normalized_signatures = {
        stamp: _cloud_content_signature(normalized[stamp].message)
        for stamp in selected_stamps
    }
    stage_data = {
        "/scan_3d_raw": cloud_stage(
            "/scan_3d_raw", raw, geometry_samples=normalized,
            geometry_note="payload-equivalent-to-/scan_3d; evaluated using lidar_link TF",
        ),
        "/scan_3d": cloud_stage("/scan_3d", normalized),
        "/obstacles_cloud": cloud_stage("/obstacles_cloud", obstacle_cloud),
        "/scan": {
            "topic": "/scan",
            "matched_count": len(selected_stamps),
            "frame_ids": sorted({scan_input[stamp].message.header.frame_id for stamp in selected_stamps}),
            "metadata_signatures": {
                str(stamp): list(_scan_metadata_signature(scan_input[stamp].message))
                for stamp in selected_stamps
            },
            "latency_vs_raw": _stage_latency(raw, scan_input, selected_stamps),
            "geometry": _stage_scan_summary(
                scan_input, selected_stamps, node.poses, obstacles, raw_support
            ),
        },
        "/scan_clean": {
            "topic": "/scan_clean",
            "matched_count": len(selected_stamps),
            "frame_ids": sorted({scan_clean[stamp].message.header.frame_id for stamp in selected_stamps}),
            "metadata_signatures": {
                str(stamp): list(_scan_metadata_signature(scan_clean[stamp].message))
                for stamp in selected_stamps
            },
            "latency_vs_raw": _stage_latency(raw, scan_clean, selected_stamps),
            "geometry": _stage_scan_summary(
                scan_clean, selected_stamps, node.poses, obstacles, clean_support
            ),
        },
    }
    projection_oracle = _projection_oracle(node, selected_stamps, obstacles)
    return {
        "schema_version": 2,
        "topic_order": [
            "/scan_3d_raw", "/scan_3d", "/obstacles_cloud", "/scan", "/scan_clean",
        ],
        "complete_chain_count": len(complete_stamps),
        "evaluated_chain_count": len(selected_stamps),
        "evaluated_stamp_ns": list(selected_stamps),
        "edge_losses": _edge_losses({
            "/scan_3d_raw": node.raw_clouds,
            "/scan_3d": node.normalized_clouds,
            "/obstacles_cloud": node.obstacle_clouds,
            "/scan": node.scan_input,
            "/scan_clean": node.scans,
        }),
        "stamp_preserved_across_chain": all(
            stamp in raw and stamp in normalized and stamp in obstacle_cloud
            and stamp in scan_input and stamp in scan_clean
            for stamp in selected_stamps
        ),
        "raw_to_normalized_content_equal": all(
            raw_signatures[stamp] == normalized_signatures[stamp]
            for stamp in selected_stamps
        ),
        "scan_to_clean_metadata_equal": scan_metadata_equal,
        "scan_to_clean_beam_support_equal": scan_support_equal,
        "common_scan_beam_count": sum(len(raw_support.get(stamp, ())) for stamp in selected_stamps),
        "common_scan_beam_support": {
            str(stamp): list(raw_support.get(stamp, ()))
            for stamp in selected_stamps
        },
        "projection_oracle": projection_oracle,
        "stages": stage_data,
    }


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
        runtime.wait(
            "complete perception stage chains",
            lambda: len(_complete_stage_stamps(node)) >= 10,
            args.timeout,
            observe=lambda: {
                "raw_clouds": len(node.raw_clouds),
                "normalized_clouds": len(node.normalized_clouds),
                "obstacle_clouds": len(node.obstacle_clouds),
                "scan": len(node.scan_input),
                "scan_clean": len(node.scans),
                "complete_chains": len(_complete_stage_stamps(node)),
            },
        )
        complete_stamps = _complete_stage_stamps(node)
        stage_lineage = _stage_lineage(node, obstacles, complete_stamps)
        validate_stage_lineage(stage_lineage)
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
        temporal_offsets_s = [
            round(-0.30 + 0.02 * index, 2) for index in range(31)
        ]
        temporal_sweep = summarize_temporal_offset_sweep(
            [(timed_scan.stamp_s, timed_scan.message) for timed_scan in node.scans[-20:]],
            node.poses,
            obstacles,
            temporal_offsets_s,
            range_min_m=0.0,
            range_max_m=20.0,
        )
        measured_sweep = [
            entry for entry in temporal_sweep
            if entry["median_rmse_m"] is not None
        ]
        baseline_sweep = next(
            (entry for entry in measured_sweep if entry["offset_s"] == 0.0),
            None,
        )
        comparable_sweep = []
        if baseline_sweep is not None:
            support = (
                baseline_sweep["paired_scan_count"],
                baseline_sweep["scored_scan_count"],
                baseline_sweep["scored_beam_count"],
                baseline_sweep["scored_beam_support"],
            )
            comparable_sweep = [
                entry for entry in measured_sweep
                if (
                    entry["paired_scan_count"],
                    entry["scored_scan_count"],
                    entry["scored_beam_count"],
                    entry["scored_beam_support"],
                ) == support
            ]
        best_temporal_offset_s = None
        if comparable_sweep:
            best_temporal_offset_s = min(
                comparable_sweep,
                key=lambda entry: (
                    float(entry["median_rmse_m"]), abs(float(entry["offset_s"])),
                ),
            )["offset_s"]
        tf_summary = _pose_divergence(
            node.tf_poses, node.poses, pose_count_key="tf_pose_count"
        )
        if (
            tf_summary["status"] != "measured"
            or int(tf_summary["tf_pose_count"]) < 2
            or int(tf_summary["paired_count"]) < 2
        ):
            raise RuntimeError(
                "dynamic TF odom -> base_footprint did not provide "
                "at least two timestamp-paired samples"
            )
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
            "tf_topic": "/tf",
            "tf_parent_frame": "odom",
            "tf_child_frame": "base_footprint",
            "scan_count": len(node.scans),
            "odom_count": node.odom_received,
            "localization_vs_raw": _pose_divergence(node.local_poses, node.poses),
            "pose_matched_scan_count": pose_matched,
            "pose_unmatched_scan_count": pose_unmatched,
            "temporal_offset_sweep_s": temporal_offsets_s,
            "best_temporal_offset_s": best_temporal_offset_s,
            "best_temporal_offset_support": (
                {
                    "paired_scan_count": baseline_sweep["paired_scan_count"],
                    "scored_scan_count": baseline_sweep["scored_scan_count"],
                    "scored_beam_count": baseline_sweep["scored_beam_count"],
                    "scored_beam_support": baseline_sweep["scored_beam_support"],
                }
                if baseline_sweep is not None else None
            ),
            "temporal_offset_sweep": temporal_sweep,
            "tf_vs_raw": tf_summary,
            "stage_lineage": stage_lineage,
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
        stage_metrics_path = args.metrics_path.with_name(
            "obstacle_drag_stage_metrics.json"
        )
        stage_metrics_path.write_text(
            json.dumps({
                "source_sha": report["source_sha"],
                "world": report["world"],
                "geometry_fixture": report["geometry_fixture"],
                **stage_lineage,
            }, indent=2) + "\n", encoding="utf-8"
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
            "raw_cloud_count": len(node.raw_clouds),
            "normalized_cloud_count": len(node.normalized_clouds),
            "obstacle_cloud_count": len(node.obstacle_clouds),
            "scan_input_count": len(node.scan_input),
            "complete_chain_count": len(_complete_stage_stamps(node)),
            "pose_count": len(node.poses),
            "local_pose_count": len(node.local_poses),
            "tf_pose_count": len(node.tf_poses),
            "metric_samples": [
                metrics.sample_count for _, metrics in timed_metrics
            ],
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
