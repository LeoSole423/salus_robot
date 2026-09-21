"""Pure metrics for comparing intermediate LiDAR pipeline representations."""

from __future__ import annotations

import hashlib
import math
from typing import Iterable, Mapping, Sequence

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


def pointcloud_payload_signature(message) -> str:
    """Hash PointCloud2 payload fields while excluding its message header."""
    fields = tuple(
        (field.name, field.offset, field.datatype, field.count)
        for field in message.fields
    )
    descriptor = repr((
        message.height, message.width, fields, message.is_bigendian,
        message.point_step, message.row_step, message.is_dense,
    )).encode("utf-8")
    digest = hashlib.sha256()
    digest.update(descriptor)
    digest.update(bytes(message.data))
    return digest.hexdigest()


def validate_stage_lineage(
    lineage: Mapping[str, object], *, minimum_chains: int = 10,
) -> None:
    """Raise when a stage capture cannot support its declared invariants."""
    errors = []
    complete_count = lineage.get("complete_chain_count", 0)
    evaluated_count = lineage.get("evaluated_chain_count", 0)
    if not isinstance(complete_count, int) or complete_count < minimum_chains:
        errors.append(f"complete_chain_count<{minimum_chains}")
    if not isinstance(evaluated_count, int) or evaluated_count < minimum_chains:
        errors.append(f"evaluated_chain_count<{minimum_chains}")
    for field in (
        "stamp_preserved_across_chain",
        "raw_to_normalized_content_equal",
        "scan_to_clean_metadata_equal",
    ):
        if lineage.get(field) is not True:
            errors.append(field)
    common_beam_count = lineage.get("common_scan_beam_count")
    if not isinstance(common_beam_count, int) or common_beam_count <= 0:
        errors.append("common_scan_beam_count<=0")

    stages = lineage.get("stages")
    expected_topics = (
        "/scan_3d_raw", "/scan_3d", "/obstacles_cloud", "/scan", "/scan_clean",
    )
    if not isinstance(stages, Mapping):
        errors.append("stages_missing")
    else:
        for topic in expected_topics:
            stage = stages.get(topic)
            if not isinstance(stage, Mapping):
                errors.append(f"stage_missing:{topic}")
                continue
            geometry = stage.get("geometry")
            if (
                not isinstance(geometry, Mapping)
                or geometry.get("status") != "measured"
            ):
                errors.append(f"geometry_unmeasured:{topic}")
            if topic not in ("/scan", "/scan_clean"):
                if not isinstance(geometry, Mapping) or geometry.get(
                    "transform_missing_count", 1
                ) != 0:
                    errors.append(f"geometry_transform_missing:{topic}")
            elif not isinstance(geometry, Mapping) or not isinstance(
                geometry.get("paired_count"), int
            ) or geometry["paired_count"] <= 0:
                errors.append(f"scan_geometry_unpaired:{topic}")
    if errors:
        raise ValueError("invalid stage lineage: " + ", ".join(errors))


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
