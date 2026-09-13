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
    errors = []
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
        errors.append(abs(observed - expected))
    if not errors:
        return StaticScanMetrics(0, None, None)
    return StaticScanMetrics(
        sample_count=len(errors),
        scan_static_error_rmse_m=math.sqrt(sum(error * error for error in errors) / len(errors)),
        scan_static_error_p95_m=_percentile(errors, 0.95),
    )
