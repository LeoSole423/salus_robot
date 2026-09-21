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


def project_pointcloud_to_scan(
    points: Iterable[tuple[float, float, float]],
    *,
    angle_min: float,
    angle_max: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    min_height: float,
    max_height: float,
    use_inf: bool = True,
    inf_epsilon: float = 1.0,
) -> tuple[float, ...]:
    """
    Reproduce the Humble pointcloud_to_laserscan projection semantics.

    This is deliberately a pure evaluator.  It mirrors the upstream node's
    filtering order, inclusive bounds, nearest-point bin reduction and empty
    bin representation without importing ROS or changing the runtime node.
    """
    if not math.isfinite(angle_increment) or angle_increment <= 0.0:
        raise ValueError("angle_increment must be finite and positive")
    if angle_max < angle_min:
        raise ValueError("angle_max must not be below angle_min")
    ranges_size = math.ceil((angle_max - angle_min) / angle_increment)
    if ranges_size <= 0:
        raise ValueError("projection must contain at least one beam")
    empty = float("inf") if use_inf else float(range_max) + inf_epsilon
    ranges = [empty] * ranges_size
    for point in points:
        x_m, y_m, z_m = (float(value) for value in point)
        if math.isnan(x_m) or math.isnan(y_m) or math.isnan(z_m):
            continue
        if z_m > max_height or z_m < min_height:
            continue
        range_m = math.hypot(x_m, y_m)
        if range_m < range_min or range_m > range_max:
            continue
        angle = math.atan2(y_m, x_m)
        if angle < angle_min or angle > angle_max:
            continue
        index = int((angle - angle_min) / angle_increment)
        if index < 0 or index >= ranges_size:
            raise ValueError(
                "accepted angle maps outside the projection range: "
                f"angle={angle!r}, index={index}, size={ranges_size}"
            )
        if range_m < ranges[index]:
            ranges[index] = range_m
    return tuple(ranges)


def compare_scan_projection(
    oracle_ranges: Sequence[float],
    actual_ranges: Sequence[float],
    *,
    tolerance_m: float = 1.0e-5,
    worst_limit: int = 5,
) -> dict[str, object]:
    """Compare oracle and runtime ranges without turning mismatch into a gate."""
    if len(oracle_ranges) != len(actual_ranges):
        raise ValueError(
            "oracle and runtime scans have different beam counts: "
            f"{len(oracle_ranges)} != {len(actual_ranges)}"
        )
    if not math.isfinite(tolerance_m) or tolerance_m < 0.0:
        raise ValueError("tolerance_m must be finite and non-negative")
    finite_deltas = []
    finite_infinite_mismatches = 0
    invalid_actual_count = 0
    exact_finite_count = 0
    common_finite_indices = []
    worst = []
    for index, (expected, actual) in enumerate(zip(oracle_ranges, actual_ranges)):
        expected_value, actual_value = float(expected), float(actual)
        expected_finite = math.isfinite(expected_value)
        actual_finite = math.isfinite(actual_value)
        expected_state = (
            "finite" if expected_finite
            else "infinite" if math.isinf(expected_value)
            else "invalid"
        )
        actual_state = (
            "finite" if actual_finite
            else "infinite" if math.isinf(actual_value)
            else "invalid"
        )
        if actual_state == "invalid":
            invalid_actual_count += 1
        if expected_finite and actual_finite:
            delta = abs(expected_value - actual_value)
            finite_deltas.append(delta)
            common_finite_indices.append(index)
            if delta <= tolerance_m:
                exact_finite_count += 1
            worst.append({"beam_index": index, "abs_delta_m": delta})
        elif expected_state != actual_state:
            finite_infinite_mismatches += 1
    worst.sort(key=lambda item: float(item["abs_delta_m"]), reverse=True)
    return {
        "beam_count": len(oracle_ranges),
        "finite_bin_count": sum(math.isfinite(float(value)) for value in oracle_ranges),
        "actual_finite_bin_count": sum(math.isfinite(float(value)) for value in actual_ranges),
        "finite_infinite_mismatch_count": finite_infinite_mismatches,
        "invalid_actual_count": invalid_actual_count,
        "finite_delta_count": len(finite_deltas),
        "finite_agreement_count": exact_finite_count,
        "finite_agreement_ratio": (
            exact_finite_count / len(finite_deltas) if finite_deltas else None
        ),
        "common_finite_support": common_finite_indices,
        "range_delta_m": {
            "status": "measured" if finite_deltas else "insufficient_data",
            "median": _percentile(finite_deltas, 0.50) if finite_deltas else None,
            "p95": _percentile(finite_deltas, 0.95) if finite_deltas else None,
            "max": max(finite_deltas) if finite_deltas else None,
        },
        "tolerance_m": tolerance_m,
        "worst_bins": worst[:worst_limit],
    }


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
    projection_oracle = lineage.get("projection_oracle")
    if not isinstance(projection_oracle, Mapping):
        errors.append("projection_oracle_missing")
    elif (
        projection_oracle.get("status") != "measured"
        or not isinstance(projection_oracle.get("paired_count"), int)
        or projection_oracle["paired_count"] < minimum_chains
    ):
        errors.append(f"projection_oracle_pairs<{minimum_chains}")
    else:
        common_support = projection_oracle.get("common_finite_support_count")
        if not isinstance(common_support, int) or common_support <= 0:
            errors.append("projection_oracle_support<=0")
        geometry_pairs = projection_oracle.get("geometry_paired_count")
        if not isinstance(geometry_pairs, int) or geometry_pairs < minimum_chains:
            errors.append(f"projection_oracle_geometry_pairs<{minimum_chains}")
        range_delta = projection_oracle.get("range_delta_m")
        if not isinstance(range_delta, Mapping) or range_delta.get(
            "status"
        ) != "measured":
            errors.append("projection_oracle_delta_unmeasured")

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
            ) or geometry["paired_count"] < minimum_chains:
                errors.append(
                    f"scan_geometry_pairs<{minimum_chains}:{topic}"
                )
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
