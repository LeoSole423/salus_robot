"""Pure metrics for comparing intermediate LiDAR pipeline representations."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

from .models import Pose2D
from .static_scan_metrics import StaticObstacle, _percentile


def exact_common_stamps(stage_stamps: Sequence[Iterable[int]]) -> tuple[int, ...]:
    """Return stamps present in every stage, sorted and without duplicates."""
    if not stage_stamps:
        return ()
    sets = [set(stamps) for stamps in stage_stamps]
    return tuple(sorted(set.intersection(*sets)))


def beam_support_is_identical(
    first: Iterable[int], second: Iterable[int],
) -> bool:
    """Compare scored beam populations independently of ordering."""
    return set(first) == set(second)


def _point_box_distance_xy(x_m: float, y_m: float, obstacle: StaticObstacle) -> float:
    """Return horizontal distance to the boundary of an axis-aligned box."""
    half_x, half_y = obstacle.size_x_m / 2.0, obstacle.size_y_m / 2.0
    dx = abs(x_m - obstacle.x_m) - half_x
    dy = abs(y_m - obstacle.y_m) - half_y
    if dx > 0.0 and dy > 0.0:
        return math.hypot(dx, dy)
    if dx > 0.0:
        return dx
    if dy > 0.0:
        return dy
    return min(-dx, -dy)


def transform_points_to_odom(
    points: Iterable[tuple[float, float]], pose: Pose2D,
) -> tuple[tuple[float, float], ...]:
    """Apply the planar raw-odometry pose to points in base footprint."""
    cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    return tuple(
        (
            pose.x_m + cosine * x_m - sine * y_m,
            pose.y_m + sine * x_m + cosine * y_m,
        )
        for x_m, y_m in points
    )


def summarize_point_geometry(
    points: Iterable[tuple[float, float]],
    obstacles: Sequence[StaticObstacle],
    *,
    neighborhood_m: float = 0.75,
) -> dict[str, object]:
    """Summarize finite points near known static obstacle surfaces."""
    if not math.isfinite(neighborhood_m) or neighborhood_m <= 0.0:
        raise ValueError("neighborhood_m must be finite and positive")
    distances = []
    for x_m, y_m in points:
        x_m, y_m = float(x_m), float(y_m)
        if not math.isfinite(x_m) or not math.isfinite(y_m):
            continue
        distance = min(
            _point_box_distance_xy(x_m, y_m, obstacle)
            for obstacle in obstacles
        )
        if distance <= neighborhood_m:
            distances.append(distance)
    if not distances:
        return {
            "status": "insufficient_data",
            "point_count": 0,
            "p95_surface_error_m": None,
            "max_surface_error_m": None,
        }
    return {
        "status": "measured",
        "point_count": len(distances),
        "p95_surface_error_m": _percentile(distances, 0.95),
        "max_surface_error_m": max(distances),
    }
