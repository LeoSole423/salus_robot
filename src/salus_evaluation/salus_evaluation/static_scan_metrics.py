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
    for index, observed in enumerate(ranges):
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
    )
