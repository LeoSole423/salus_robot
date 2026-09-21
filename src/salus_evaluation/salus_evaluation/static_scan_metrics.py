"""Geometry-only metrics for comparing a clean simulated scan with known boxes."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import yaml

from .models import Pose2D


@dataclass(frozen=True)
class StaticObstacle:
    """Axis-aligned static box in the fixed evaluation frame."""

    name: str
    x_m: float
    y_m: float
    size_x_m: float
    size_y_m: float


@dataclass(frozen=True)
class StaticScanMetrics:
    """Absolute range error for beams expected to hit known geometry."""

    sample_count: int
    scan_static_error_rmse_m: float | None
    scan_static_error_p95_m: float | None
    max_error_m: float | None = None
    worst_beam_indices: tuple[int, ...] = ()
    worst_errors_m: tuple[float, ...] = ()
    scored_beam_indices: tuple[int, ...] = ()


def summarize_temporal_offset_sweep(
    timed_scans: Iterable[tuple[float, object]],
    pose_samples: Sequence[tuple[float, Pose2D]],
    obstacles: Iterable[StaticObstacle],
    offsets_s: Iterable[float],
    *,
    range_min_m: float = 0.0,
    range_max_m: float = math.inf,
    beam_indices: Iterable[int] | None = None,
) -> list[dict[str, object]]:
    """
    Report scan geometry error for a bounded pose timestamp sweep.

    Positive offsets evaluate the pose after the scan timestamp; negative
    offsets evaluate it before.  This is deliberately diagnostic: it does not
    select or apply a runtime correction and it never extrapolates a pose.
    ``timed_scans`` contains ``(stamp_s, LaserScan-like message)`` pairs so the
    pure metric remains independent of ROS node state.

    Every offset is evaluated on the same scan set and the same per-scan beam
    intersection.  This prevents a candidate offset from looking better only
    because its pose makes a different subset of beams scoreable.
    """
    scans = tuple((float(stamp_s), scan) for stamp_s, scan in timed_scans)
    obstacles = tuple(obstacles)
    offsets = tuple(float(raw_offset_s) for raw_offset_s in offsets_s)
    if any(not math.isfinite(offset_s) for offset_s in offsets):
        raise ValueError("temporal offsets must be finite")
    selected_beams = None if beam_indices is None else frozenset(
        int(index) for index in beam_indices
    )

    # Restrict every offset to the same scans and the intersection of the
    # beams that are valid for every pose.  Otherwise a timestamp sweep can
    # appear to improve merely by scoring a different subset of the cloud.
    comparable_scans: list[tuple[float, object, tuple[int, ...]]] = []
    for stamp_s, scan in scans:
        poses = [
            interpolate_pose(pose_samples, stamp_s + offset_s)
            for offset_s in offsets
        ]
        if any(pose is None for pose in poses):
            continue
        supports = []
        for pose in poses:
            assert pose is not None
            metric = scan_static_error_metrics(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                pose,
                obstacles,
                range_min_m=range_min_m,
                range_max_m=range_max_m,
            )
            supports.append(set(metric.scored_beam_indices))
        common_beams = set.intersection(*supports) if supports else set()
        if selected_beams is not None:
            common_beams.intersection_update(selected_beams)
        comparable_scans.append((stamp_s, scan, tuple(sorted(common_beams))))

    results: list[dict[str, object]] = []
    for offset_s in offsets:
        metrics: list[StaticScanMetrics] = []
        for stamp_s, scan, common_beams in comparable_scans:
            pose = interpolate_pose(pose_samples, stamp_s + offset_s)
            assert pose is not None
            current = scan_static_error_metrics(
                scan.ranges,
                scan.angle_min,
                scan.angle_increment,
                pose,
                obstacles,
                range_min_m=range_min_m,
                range_max_m=range_max_m,
                beam_indices=common_beams,
            )
            if current.sample_count:
                metrics.append(current)
        support = [
            {"stamp_s": stamp_s, "beam_indices": list(common_beams)}
            for stamp_s, _, common_beams in comparable_scans
            if common_beams
        ]
        if not metrics:
            results.append({
                "offset_s": offset_s,
                "paired_scan_count": len(comparable_scans),
                "scored_scan_count": 0,
                "scored_beam_count": 0,
                "scored_beam_support": support,
                "median_rmse_m": None,
                "max_rmse_m": None,
                "max_p95_m": None,
                "max_beam_m": None,
            })
            continue
        rmse_values = sorted(
            float(metric.scan_static_error_rmse_m) for metric in metrics
        )
        middle = (len(rmse_values) - 1) * 0.5
        low, high = math.floor(middle), math.ceil(middle)
        median_rmse = rmse_values[low] + (rmse_values[high] - rmse_values[low]) * (
            middle - low
        )
        results.append({
            "offset_s": offset_s,
            "paired_scan_count": len(comparable_scans),
            "scored_scan_count": len(metrics),
            "scored_beam_count": sum(metric.sample_count for metric in metrics),
            "scored_beam_support": support,
            "median_rmse_m": median_rmse,
            "max_rmse_m": max(
                float(metric.scan_static_error_rmse_m) for metric in metrics
            ),
            "max_p95_m": max(
                float(metric.scan_static_error_p95_m) for metric in metrics
            ),
            "max_beam_m": max(
                float(metric.max_error_m) for metric in metrics
            ),
        })
    return results


def summarize_scan_metrics(
    timed_metrics: Iterable[tuple[float, StaticScanMetrics]],
) -> dict[str, object]:
    """
    Build the version-2 report fields for timestamped scan measurements.

    The input is sorted by ROS timestamp so that the per-scan sequence remains
    deterministic even when callback delivery order differs from message time.
    All entries are expected to contain at least one scored beam; an empty
    sequence is a measurement failure rather than a reportable zero.
    """
    ordered = sorted(
        ((float(stamp_s), metrics) for stamp_s, metrics in timed_metrics),
        key=lambda item: item[0],
    )
    if not ordered:
        raise ValueError("at least one scan metric is required")
    if any(not math.isfinite(stamp_s) for stamp_s, _ in ordered):
        raise ValueError("scan metric timestamps must be finite")
    if any(metrics.sample_count <= 0 for _, metrics in ordered):
        raise ValueError("scan metrics must contain at least one sample")

    per_scan_metrics = [
        {
            "stamp_s": stamp_s,
            "sample_count": metrics.sample_count,
            "rmse_m": metrics.scan_static_error_rmse_m,
            "p95_m": metrics.scan_static_error_p95_m,
            "max_m": metrics.max_error_m,
        }
        for stamp_s, metrics in ordered
    ]
    return {
        "samples_per_scan": [
            entry["sample_count"] for entry in per_scan_metrics
        ],
        "max_scan_static_error_rmse_m": max(
            entry["rmse_m"] for entry in per_scan_metrics
        ),
        "max_scan_static_error_p95_m": max(
            entry["p95_m"] for entry in per_scan_metrics
        ),
        "max_beam_static_error_m": max(
            entry["max_m"] for entry in per_scan_metrics
        ),
        "per_scan_metrics": per_scan_metrics,
    }


def interpolate_pose(
    pose_samples: Sequence[tuple[float, Pose2D]], stamp_s: float,
) -> Pose2D | None:
    """
    Interpolate a fixed-frame pose at a ROS message timestamp.

    Samples must be ordered by their ROS timestamp. The function deliberately
    does not extrapolate: a scan without bracketing ground truth is not paired
    with an unrelated pose.
    """
    if not math.isfinite(float(stamp_s)):
        raise ValueError("stamp_s must be finite")
    if not pose_samples:
        return None
    for index, (sample_stamp, sample_pose) in enumerate(pose_samples):
        if stamp_s == sample_stamp:
            return sample_pose
        if stamp_s < sample_stamp:
            if index == 0:
                return None
            previous_stamp, previous_pose = pose_samples[index - 1]
            span = sample_stamp - previous_stamp
            if span <= 0.0:
                raise ValueError("pose timestamps must be strictly increasing")
            fraction = (stamp_s - previous_stamp) / span
            yaw_delta = math.atan2(
                math.sin(sample_pose.yaw_rad - previous_pose.yaw_rad),
                math.cos(sample_pose.yaw_rad - previous_pose.yaw_rad),
            )
            return Pose2D(
                previous_pose.x_m + fraction * (sample_pose.x_m - previous_pose.x_m),
                previous_pose.y_m + fraction * (sample_pose.y_m - previous_pose.y_m),
                previous_pose.yaw_rad + fraction * yaw_delta,
            )
    return None


def summarize_pose_divergence(
    estimated_poses: Sequence[tuple[float, Pose2D]],
    raw_poses: Sequence[tuple[float, Pose2D]],
) -> dict[str, object]:
    """Compare a timestamped pose stream with raw odometry by interpolation."""
    position_errors: list[float] = []
    yaw_errors: list[float] = []
    for stamp_s, estimated in estimated_poses:
        raw = interpolate_pose(raw_poses, stamp_s)
        if raw is None:
            continue
        position_errors.append(math.hypot(
            estimated.x_m - raw.x_m, estimated.y_m - raw.y_m,
        ))
        yaw_errors.append(abs(math.atan2(
            math.sin(estimated.yaw_rad - raw.yaw_rad),
            math.cos(estimated.yaw_rad - raw.yaw_rad),
        )))
    if not position_errors:
        return {
            "status": "insufficient_data",
            "pose_count": len(estimated_poses),
            "paired_count": 0,
            "p95_position_error_m": None,
            "max_position_error_m": None,
            "p95_yaw_error_rad": None,
            "max_yaw_error_rad": None,
        }
    return {
        "status": "measured",
        "pose_count": len(estimated_poses),
        "paired_count": len(position_errors),
        "p95_position_error_m": _percentile(position_errors, 0.95),
        "max_position_error_m": max(position_errors),
        "p95_yaw_error_rad": _percentile(yaw_errors, 0.95),
        "max_yaw_error_rad": max(yaw_errors),
    }


def _finite_positive(value: object, label: str) -> float:
    number = float(value)
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError(f"{label} must be finite and positive")
    return number


def load_obstacle_geometry(path: str | Path) -> tuple[str, tuple[StaticObstacle, ...]]:
    """Load the small, versioned box fixture used by the obstacle-drag world."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        raise ValueError("unsupported obstacle geometry schema")
    fixed_frame = str(data.get("fixed_frame", ""))
    if not fixed_frame:
        raise ValueError("fixed_frame is required")
    obstacles = []
    for item in data.get("obstacles", []):
        if item.get("shape") != "box":
            raise ValueError("only axis-aligned box obstacles are supported")
        center, size = item["center_m"], item["size_m"]
        obstacles.append(StaticObstacle(
            name=str(item["name"]),
            x_m=float(center["x"]), y_m=float(center["y"]),
            size_x_m=_finite_positive(size["x"], f"{item['name']}.size.x"),
            size_y_m=_finite_positive(size["y"], f"{item['name']}.size.y"),
        ))
    if not obstacles:
        raise ValueError("at least one obstacle is required")
    return fixed_frame, tuple(obstacles)


def ray_box_intersection(
    origin_x_m: float, origin_y_m: float, angle_rad: float,
    obstacle: StaticObstacle,
) -> float | None:
    """Return the first non-negative distance where a ray hits an axis box."""
    direction_x, direction_y = math.cos(angle_rad), math.sin(angle_rad)
    lower = (obstacle.x_m - obstacle.size_x_m / 2.0,
             obstacle.y_m - obstacle.size_y_m / 2.0)
    upper = (obstacle.x_m + obstacle.size_x_m / 2.0,
             obstacle.y_m + obstacle.size_y_m / 2.0)
    t_min, t_max = -math.inf, math.inf
    for origin, direction, low, high in zip(
        (origin_x_m, origin_y_m), (direction_x, direction_y), lower, upper
    ):
        if abs(direction) < 1.0e-12:
            if origin < low or origin > high:
                return None
            continue
        near, far = (low - origin) / direction, (high - origin) / direction
        if near > far:
            near, far = far, near
        t_min, t_max = max(t_min, near), min(t_max, far)
        if t_min > t_max:
            return None
    if t_max < 0.0:
        return None
    return max(0.0, t_min)


def _percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low, high = math.floor(index), math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def scan_static_error_metrics(
    ranges: Iterable[float], angle_min_rad: float, angle_increment_rad: float,
    robot_pose: Pose2D, obstacles: Iterable[StaticObstacle], *,
    range_min_m: float = 0.0, range_max_m: float = math.inf,
    beam_indices: Iterable[int] | None = None,
) -> StaticScanMetrics:
    """
    Compare scan ranges with known boxes after transforming rays into ``odom``.

    ``ranges`` are from ``/scan_clean`` whose target frame is ``base_footprint``.
    The supplied robot pose is therefore the transform from that scan frame into
    the fixed geometry frame.  Only beams with a known box intersection are
    scored; unknown/free-space beams are intentionally not treated as errors.
    """
    if not math.isfinite(angle_min_rad) or not math.isfinite(angle_increment_rad):
        raise ValueError("scan angles must be finite")
    obstacles = tuple(obstacles)
    errors: list[tuple[float, int]] = []
    origin_x, origin_y = robot_pose.x_m, robot_pose.y_m
    selected_beams = None if beam_indices is None else frozenset(
        int(index) for index in beam_indices
    )
    for index, observed in enumerate(ranges):
        if selected_beams is not None and index not in selected_beams:
            continue
        observed = float(observed)
        if not math.isfinite(observed) or observed < range_min_m or observed > range_max_m:
            continue
        beam_angle = angle_min_rad + index * angle_increment_rad
        world_angle = robot_pose.yaw_rad + beam_angle
        expected_values = [
            hit for obstacle in obstacles
            if (hit := ray_box_intersection(origin_x, origin_y, world_angle, obstacle))
            is not None
            and range_min_m <= hit <= range_max_m
        ]
        if not expected_values:
            continue
        expected = min(expected_values)
        errors.append((abs(observed - expected), index))
    if not errors:
        return StaticScanMetrics(0, None, None)
    error_values = [error for error, _ in errors]
    worst = sorted(errors, reverse=True)[:3]
    return StaticScanMetrics(
        sample_count=len(error_values),
        scan_static_error_rmse_m=math.sqrt(
            sum(error * error for error in error_values) / len(error_values)
        ),
        scan_static_error_p95_m=_percentile(error_values, 0.95),
        max_error_m=max(error_values),
        worst_beam_indices=tuple(index for _, index in worst),
        worst_errors_m=tuple(error for error, _ in worst),
        scored_beam_indices=tuple(index for _, index in errors),
    )
