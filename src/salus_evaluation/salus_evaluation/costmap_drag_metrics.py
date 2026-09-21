"""
Pure metrics for detecting obstacle trails in published costmaps.

The functions in this module deliberately do not own a ROS node.  A probe can
feed them immutable grid snapshots and poses, which keeps the interpretation
testable with positive and negative synthetic controls.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

from .models import Pose2D
from .static_scan_metrics import StaticObstacle, _percentile


@dataclass(frozen=True)
class CostmapSnapshot:
    """One ``nav2_msgs/Costmap`` snapshot reduced to its fields."""

    stamp_s: float
    frame_id: str
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    width: int
    height: int
    data: tuple[int, ...]
    origin_yaw_rad: float = 0.0


@dataclass(frozen=True)
class CostmapObservation:
    """Occupied cells expressed in both fixed and robot frames."""

    stamp_s: float
    phase: str
    odom_points: tuple[tuple[float, float], ...]
    base_points: tuple[tuple[float, float], ...]


def occupied_grid_points(
    snapshot: CostmapSnapshot, *, occupied_costs: Sequence[int] = (254,),
) -> tuple[tuple[float, float], ...]:
    """
    Return centers of lethal cells in the costmap frame.

    ``nav2_msgs/Costmap`` stores unsigned Nav2 costs: 253 is an inscribed or
    inflated cell, 254 is a lethal obstacle and 255 is unknown. The default
    selects only the lethal value so inflated geometry is not mistaken for a
    sensor mark. Callers may explicitly select another cost for a separate
    diagnostic, but unknown space must never be selected implicitly.
    """
    if snapshot.width <= 0 or snapshot.height <= 0:
        return ()
    if not math.isfinite(snapshot.resolution_m) or snapshot.resolution_m <= 0.0:
        raise ValueError("costmap resolution must be finite and positive")
    expected = snapshot.width * snapshot.height
    if len(snapshot.data) != expected:
        raise ValueError(f"costmap data length {len(snapshot.data)} != {expected}")
    selected_costs = frozenset(int(cost) for cost in occupied_costs)
    if not selected_costs or any(cost < 0 or cost > 255 for cost in selected_costs):
        raise ValueError("occupied_costs must contain unsigned byte values")
    points = []
    cosine, sine = math.cos(snapshot.origin_yaw_rad), math.sin(snapshot.origin_yaw_rad)
    for index, raw_value in enumerate(snapshot.data):
        if int(raw_value) not in selected_costs:
            continue
        row, column = divmod(index, snapshot.width)
        local_x = (column + 0.5) * snapshot.resolution_m
        local_y = (row + 0.5) * snapshot.resolution_m
        points.append((
            snapshot.origin_x_m + cosine * local_x - sine * local_y,
            snapshot.origin_y_m + sine * local_x + cosine * local_y,
        ))
    return tuple(points)


def transform_costmap_points_to_odom(
    points: Iterable[tuple[float, float]], frame_id: str, pose: Pose2D,
) -> tuple[tuple[float, float], ...]:
    """Express grid points in ``odom`` for supported rolling-frame maps."""
    if frame_id == "odom":
        return tuple((float(x), float(y)) for x, y in points)
    if frame_id != "base_footprint":
        raise ValueError(f"unsupported costmap frame: {frame_id!r}")
    cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    return tuple(
        (
            pose.x_m + cosine * float(x) - sine * float(y),
            pose.y_m + sine * float(x) + cosine * float(y),
        )
        for x, y in points
    )


def transform_odom_points_to_base(
    points: Iterable[tuple[float, float]], pose: Pose2D,
) -> tuple[tuple[float, float], ...]:
    """Express fixed-frame points in the robot frame at a pose timestamp."""
    cosine, sine = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
    return tuple(
        (
            cosine * (float(x) - pose.x_m) + sine * (float(y) - pose.y_m),
            -sine * (float(x) - pose.x_m) + cosine * (float(y) - pose.y_m),
        )
        for x, y in points
    )


def _centroid(points: Sequence[tuple[float, float]]) -> tuple[float, float] | None:
    if not points:
        return None
    return (
        sum(float(x) for x, _ in points) / len(points),
        sum(float(y) for _, y in points) / len(points),
    )


def _nearest_point_distance(
    point: tuple[float, float], candidates: Sequence[tuple[float, float]],
) -> float:
    if not candidates:
        return math.inf
    return min(math.hypot(point[0] - x, point[1] - y) for x, y in candidates)


def _distance_outside_obstacle(
    point: tuple[float, float], obstacle: StaticObstacle,
) -> float:
    """Return zero inside a box and Euclidean distance outside its boundary."""
    dx = abs(point[0] - obstacle.x_m) - obstacle.size_x_m / 2.0
    dy = abs(point[1] - obstacle.y_m) - obstacle.size_y_m / 2.0
    if dx <= 0.0 and dy <= 0.0:
        return 0.0
    if dx > 0.0 and dy > 0.0:
        return math.hypot(dx, dy)
    return max(dx, dy)


def _voxel(point: tuple[float, float], resolution_m: float) -> tuple[int, int]:
    return (round(point[0] / resolution_m), round(point[1] / resolution_m))


def summarize_cohort_frame_tracking(
    observations: Sequence[CostmapObservation],
    *,
    scan_support: Sequence[tuple[float, Sequence[tuple[float, float]]]],
    target_obstacle: StaticObstacle,
    cohort_phase: str,
    measurement_phases: Sequence[str],
    witness_phase: str | None = None,
    support_tolerance_m: float = 0.20,
    stamp_tolerance_s: float = 0.25,
    dominance_fraction: float = 0.25,
    roi_margin_m: float = 0.50,
    minimum_seed_cells: int = 5,
) -> dict[str, object]:
    """
    Test whether a lethal-cell cohort follows odom or base.

    The target region is fixed by scenario geometry before measurement. The
    seed is the last observation in ``cohort_phase`` with enough lethal cells
    inside that region. By default those cells require a contemporaneous scan;
    when ``witness_phase`` is set, the scan witness must instead exist in that
    earlier phase. Matching exactly that seed in odom tests the world-fixed
    hypothesis; matching its paired base-frame coordinates tests the
    robot-attached hypothesis. A later mark is never substituted for the seed.

    The matching radius is the same measured support tolerance used to decide
    whether a cell has a sensor witness.  Classification additionally requires
    at least half of the seed and a 25 percentage-point advantage, keeping an
    ambiguous result reportable rather than forcing a causal conclusion.
    """
    if support_tolerance_m <= 0.0 or stamp_tolerance_s < 0.0:
        raise ValueError("tracking tolerances must be positive")
    if not 0.0 <= dominance_fraction <= 1.0:
        raise ValueError("dominance_fraction must be between zero and one")
    if roi_margin_m < 0.0 or minimum_seed_cells <= 0:
        raise ValueError("ROI margin and minimum seed size must be valid")
    ordered = sorted(observations, key=lambda item: float(item.stamp_s))
    support = sorted(
        ((float(stamp), tuple(points)) for stamp, points in scan_support),
        key=lambda item: item[0],
    )

    def contemporaneous_points(stamp_s: float) -> tuple[tuple[float, float], ...]:
        return tuple(
            point
            for scan_stamp, scan_points in support
            if abs(scan_stamp - stamp_s) <= stamp_tolerance_s
            for point in scan_points
        )

    witness_ready = witness_phase is None
    if witness_phase is not None:
        for observation in ordered:
            if observation.phase != witness_phase:
                continue
            scan_points = contemporaneous_points(observation.stamp_s)
            if any(
                _distance_outside_obstacle(point, target_obstacle) <= roi_margin_m
                and scan_points
                and _nearest_point_distance(point, scan_points)
                <= support_tolerance_m
                for point in observation.odom_points
            ):
                witness_ready = True
                break
    seed_observation = None
    seed_indices: tuple[int, ...] = ()
    for observation in ordered:
        if observation.phase != cohort_phase:
            continue
        if len(observation.odom_points) != len(observation.base_points):
            raise ValueError("odom/base costmap point arrays must stay aligned")
        scan_points = contemporaneous_points(observation.stamp_s)
        indices = tuple(
            index for index, point in enumerate(observation.odom_points)
            if (
                _distance_outside_obstacle(point, target_obstacle) <= roi_margin_m
                and (
                    witness_phase is not None
                    or (
                        scan_points
                        and _nearest_point_distance(point, scan_points)
                        <= support_tolerance_m
                    )
                )
            )
        )
        if len(indices) >= minimum_seed_cells:
            seed_observation, seed_indices = observation, indices

    empty = {
        "status": "insufficient_data",
        "classification": "insufficient_data",
        "matched_frame_before_clear": None,
        "target_obstacle": target_obstacle.name,
        "witness_phase": witness_phase,
        "cohort_phase": cohort_phase,
        "measurement_phases": list(measurement_phases),
        "seed_stamp_s": None,
        "seed_cell_count": 0,
        "usable_observation_count": 0,
        "unsupported_observation_count": 0,
        "world_fixed_match_fraction_median": None,
        "base_attached_match_fraction_median": None,
        "samples": [],
    }
    if seed_observation is None or not witness_ready:
        return empty
    seed_odom = tuple(
        seed_observation.odom_points[index] for index in seed_indices
    )
    seed_base = tuple(
        seed_observation.base_points[index] for index in seed_indices
    )
    target_phases = frozenset(measurement_phases)
    minimum_match_fraction = 0.50
    samples = []
    for observation in ordered:
        if observation.stamp_s <= seed_observation.stamp_s:
            continue
        if observation.phase not in target_phases or not observation.odom_points:
            continue
        if len(observation.odom_points) != len(observation.base_points):
            raise ValueError("odom/base costmap point arrays must stay aligned")
        world_distances = tuple(
            _nearest_point_distance(point, observation.odom_points)
            for point in seed_odom
        )
        base_distances = tuple(
            _nearest_point_distance(point, observation.base_points)
            for point in seed_base
        )
        world_fraction = sum(
            distance <= support_tolerance_m for distance in world_distances
        ) / len(seed_odom)
        base_fraction = sum(
            distance <= support_tolerance_m for distance in base_distances
        ) / len(seed_base)
        state = "unmatched"
        if (
            world_fraction >= minimum_match_fraction
            and world_fraction - base_fraction >= dominance_fraction
        ):
            state = "world_fixed"
        elif (
            base_fraction >= minimum_match_fraction
            and base_fraction - world_fraction >= dominance_fraction
        ):
            state = "base_attached"
        elif max(world_fraction, base_fraction) >= minimum_match_fraction:
            state = "ambiguous"
        samples.append({
            "stamp_s": float(observation.stamp_s),
            "phase": observation.phase,
            "state": state,
            "world_fixed_match_fraction": world_fraction,
            "base_attached_match_fraction": base_fraction,
            "world_fixed_residual_p95_m": _percentile(world_distances, 0.95),
            "base_attached_residual_p95_m": _percentile(base_distances, 0.95),
        })
    if not samples:
        return {
            **empty,
            "seed_stamp_s": float(seed_observation.stamp_s),
            "seed_cell_count": len(seed_indices),
        }
    matched_states = [
        item["state"] for item in samples
        if item["state"] in ("world_fixed", "base_attached")
    ]
    matched_frame = (
        max(set(matched_states), key=matched_states.count)
        if matched_states else None
    )
    terminal_cleared = samples[-1]["state"] == "unmatched" and bool(matched_states)
    classification = "cleared" if terminal_cleared else (
        matched_frame or samples[-1]["state"]
    )
    world_median = _percentile(
        [float(item["world_fixed_match_fraction"]) for item in samples], 0.50
    )
    base_median = _percentile(
        [float(item["base_attached_match_fraction"]) for item in samples], 0.50
    )
    return {
        "status": "measured",
        "classification": classification,
        "matched_frame_before_clear": matched_frame,
        "target_obstacle": target_obstacle.name,
        "witness_phase": witness_phase,
        "cohort_phase": cohort_phase,
        "measurement_phases": list(measurement_phases),
        "support_tolerance_m": support_tolerance_m,
        "stamp_tolerance_s": stamp_tolerance_s,
        "roi_margin_m": roi_margin_m,
        "minimum_seed_cells": minimum_seed_cells,
        "minimum_match_fraction": minimum_match_fraction,
        "dominance_fraction": dominance_fraction,
        "seed_stamp_s": float(seed_observation.stamp_s),
        "seed_cell_count": len(seed_indices),
        "seed_centroid_odom": list(_centroid(seed_odom) or ()),
        "usable_observation_count": len(samples),
        "world_fixed_match_fraction_median": world_median,
        "base_attached_match_fraction_median": base_median,
        "samples": samples,
    }


def summarize_costmap_observations(
    observations: Sequence[CostmapObservation],
    obstacles: Sequence[StaticObstacle],
    *,
    scan_support: Sequence[tuple[float, Sequence[tuple[float, float]]]] = (),
    neighborhood_m: float = 3.0,
    support_tolerance_m: float = 0.20,
    voxel_resolution_m: float = 0.10,
    required_phases: Sequence[str] = (),
    cohort_phase: str | None = None,
    horizon_s: float | None = None,
    horizon_tolerance_s: float = 0.0,
    require_scan_support: bool = False,
) -> dict[str, object]:
    """
    Summarize trail width, persistence and frame-attached motion.

    ``scan_support`` contains timestamped ``/scan_clean`` points already
    expressed in ``odom``.  Unsupported cells are report-only evidence: a
    raytracing field-of-view can legitimately leave a mark outside the next
    scan, so this function never labels a cell as a safety failure.
    """
    ordered = sorted(observations, key=lambda item: float(item.stamp_s))
    phase_counts = {
        phase: sum(1 for item in ordered if item.phase == phase)
        for phase in sorted({item.phase for item in ordered})
    }
    missing_phases = [
        phase for phase in required_phases if phase_counts.get(phase, 0) == 0
    ]
    phase_occupied_counts = {
        phase: sum(1 for item in ordered
                   if item.phase == phase and item.odom_points)
        for phase in phase_counts
    }
    missing_occupied_phases = [
        phase for phase in required_phases
        if phase_occupied_counts.get(phase, 0) == 0
    ]
    if not ordered or not obstacles:
        return {
            "status": "insufficient_data",
            "observation_count": len(ordered),
            "valid_occupied_observation_count": 0,
            "phase_counts": phase_counts,
            "missing_phases": missing_phases,
            "phase_occupied_counts": phase_occupied_counts,
            "missing_occupied_phases": missing_occupied_phases,
            "cohort_phase": cohort_phase,
            "cohort_observation_count": 0,
            "cohort_supported_cell_count": 0,
            "scan_support_count": len(scan_support),
            "scan_supported_point_count": 0,
            "horizon_s": horizon_s,
            "measurement_start_s": None,
            "measurement_end_s": None,
            "measurement_coverage_s": 0.0,
            "trail_width_p95_m": None,
            "ghost_persistence_s": None,
            "centroids": [],
            "classification": "insufficient_data",
        }
    if neighborhood_m <= 0.0 or support_tolerance_m < 0.0:
        raise ValueError("metric bounds must be non-negative")
    if horizon_s is not None and (not math.isfinite(horizon_s) or horizon_s <= 0.0):
        raise ValueError("horizon_s must be finite and positive")
    if (
        not math.isfinite(horizon_tolerance_s)
        or horizon_tolerance_s < 0.0
        or (horizon_s is not None and horizon_tolerance_s >= horizon_s)
    ):
        raise ValueError("horizon_tolerance_s must be finite and shorter than horizon")
    centroids = []
    for observation in ordered:
        centroid_odom = _centroid(observation.odom_points)
        centroid_base = _centroid(observation.base_points)
        centroids.append({
            "stamp_s": float(observation.stamp_s),
            "phase": observation.phase,
            "odom_xy": list(centroid_odom) if centroid_odom else None,
            "base_xy": list(centroid_base) if centroid_base else None,
            "occupied_count": len(observation.odom_points),
        })
    all_support_by_stamp = sorted(
        ((float(stamp), tuple(points)) for stamp, points in scan_support),
        key=lambda item: item[0],
    )

    def supported_points(
        observation: CostmapObservation,
    ) -> tuple[tuple[float, float], ...]:
        return tuple(
            point for point in observation.odom_points
            if any(
                abs(stamp - observation.stamp_s) <= 0.25
                and _nearest_point_distance(point, scan_points)
                <= support_tolerance_m
                for stamp, scan_points in all_support_by_stamp
            )
        )

    valid_occupied = [item for item in ordered if item.odom_points]
    cohort = [
        item for item in valid_occupied
        if cohort_phase is None or item.phase == cohort_phase
    ]
    supported_cohort = [
        (item, supported_points(item)) for item in cohort
    ]
    supported_cohort = [
        (item, points) for item, points in supported_cohort if points
    ]
    if require_scan_support:
        seed_observation, seed_points = (
            supported_cohort[-1] if supported_cohort else (None, ())
        )
    else:
        seed_observation = cohort[-1] if cohort else None
        seed_points = seed_observation.odom_points if seed_observation else ()
    measurement_start_s = (
        seed_observation.stamp_s
        if seed_observation is not None and horizon_s is not None else None
    )
    measurement_end_s = (
        seed_observation.stamp_s + horizon_s
        if seed_observation is not None and horizon_s is not None else None
    )
    measurement_observations = (
        [
            item for item in ordered
            if measurement_start_s <= item.stamp_s <= measurement_end_s
        ]
        if measurement_start_s is not None and measurement_end_s is not None
        else ([] if horizon_s is not None else list(ordered))
    )
    measurement_coverage_s = (
        max(0.0, measurement_observations[-1].stamp_s - measurement_start_s)
        if measurement_start_s is not None and measurement_observations else 0.0
    )
    coverage_ready = (
        horizon_s is None
        or measurement_coverage_s >= horizon_s - horizon_tolerance_s
    )
    measurement_phase_occupied_counts = {
        phase: sum(
            1 for item in measurement_observations
            if item.phase == phase and item.odom_points
        )
        for phase in phase_counts
    }
    measurement_missing_occupied_phases = [
        phase for phase in required_phases
        if measurement_phase_occupied_counts.get(phase, 0) == 0
    ]
    measurement_valid_occupied = [
        item for item in measurement_observations if item.odom_points
    ]
    widths = []
    for observation in measurement_observations:
        for point in observation.odom_points:
            distance = min(
                _distance_outside_obstacle(point, obstacle)
                for obstacle in obstacles
            )
            if distance <= neighborhood_m:
                widths.append(distance)
    support_by_stamp = (
        [
            item for item in all_support_by_stamp
            if measurement_start_s <= item[0] <= measurement_end_s
        ]
        if measurement_start_s is not None and measurement_end_s is not None
        else all_support_by_stamp
    )
    scan_supported_point_count = sum(
        1 for _, scan_points in support_by_stamp
        for point in scan_points
        if min(
            _distance_outside_obstacle(point, obstacle)
            for obstacle in obstacles
        ) <= support_tolerance_m
    )
    ghost_persistence = []
    for point in seed_points:
        key = _voxel(point, voxel_resolution_m)
        unsupported_start: float | None = None
        last_unsupported: float | None = None
        for observation in measurement_observations:
            if seed_observation and observation.stamp_s <= seed_observation.stamp_s:
                continue
            occupied_keys = {
                _voxel(candidate, voxel_resolution_m)
                for candidate in observation.odom_points
            }
            if key not in occupied_keys:
                if unsupported_start is not None and last_unsupported is not None:
                    ghost_persistence.append(last_unsupported - unsupported_start)
                    unsupported_start = None
                    last_unsupported = None
                continue
            supported = any(
                stamp <= observation.stamp_s
                and _nearest_point_distance(point, scan_points) <= support_tolerance_m
                for stamp, scan_points in support_by_stamp
                if abs(stamp - observation.stamp_s) <= 0.25
            )
            if supported:
                if unsupported_start is not None and last_unsupported is not None:
                    ghost_persistence.append(last_unsupported - unsupported_start)
                unsupported_start = None
                last_unsupported = None
            else:
                if unsupported_start is None:
                    unsupported_start = float(observation.stamp_s)
                last_unsupported = float(observation.stamp_s)
        if unsupported_start is not None and last_unsupported is not None:
            ghost_persistence.append(last_unsupported - unsupported_start)

    seed_supported_cell_count = (
        len(seed_points) if require_scan_support
        else len(supported_points(seed_observation))
        if seed_observation is not None else 0
    )
    # The scan stream is an independent temporal witness for the capture.  A
    # particular lethal cell can legitimately miss that witness because the
    # rolling costmap and the scan are sampled at different instants, so the
    # measurement gate requires a timestamped scan stream that sees the known
    # obstacle while retaining per-cell support as diagnostic evidence.
    support_ready = seed_supported_cell_count > 0
    measurement_ready = (
        bool(widths)
        and bool(measurement_valid_occupied)
        and not measurement_missing_occupied_phases
        and (cohort_phase is None or seed_observation is not None)
        and coverage_ready
        and (not require_scan_support or support_ready)
    )
    classification = "insufficient_data"
    nonempty = [item for item in measurement_observations if item.base_points]
    if required_phases:
        nonempty = [item for item in nonempty if item.phase in required_phases]
    if (
        len(nonempty) >= 1
        and measurement_observations
        and not measurement_observations[-1].odom_points
    ):
        classification = "cleared"
    elif len(nonempty) >= 2:
        first, last = nonempty[0], nonempty[-1]
        first_odom = _centroid(first.odom_points)
        last_odom = _centroid(last.odom_points)
        first_base = _centroid(first.base_points)
        last_base = _centroid(last.base_points)
        assert first_odom and last_odom and first_base and last_base
        odom_shift = math.hypot(
            last_odom[0] - first_odom[0], last_odom[1] - first_odom[1]
        )
        base_shift = math.hypot(
            last_base[0] - first_base[0], last_base[1] - first_base[1]
        )
        if odom_shift <= support_tolerance_m:
            classification = "world_fixed"
        elif base_shift <= support_tolerance_m:
            classification = "base_attached"
        else:
            classification = "insufficient_data"
    return {
        "status": (
            "measured"
            if measurement_ready
            else "insufficient_data"
        ),
        "observation_count": len(ordered),
        "valid_occupied_observation_count": len(valid_occupied),
        "phase_counts": phase_counts,
        "missing_phases": missing_phases,
        "phase_occupied_counts": phase_occupied_counts,
        "missing_occupied_phases": measurement_missing_occupied_phases,
        "measurement_phase_occupied_counts": measurement_phase_occupied_counts,
        "cohort_phase": cohort_phase,
        "cohort_observation_count": len(cohort),
        "cohort_supported_cell_count": seed_supported_cell_count,
        "scan_support_count": len(support_by_stamp),
        "total_scan_support_count": len(all_support_by_stamp),
        "scan_supported_point_count": scan_supported_point_count,
        "horizon_s": horizon_s,
        "measurement_start_s": measurement_start_s,
        "measurement_end_s": measurement_end_s,
        "measurement_coverage_s": measurement_coverage_s,
        "trail_width_p95_m": _percentile(widths, 0.95) if widths else None,
        "trail_width_sample_count": len(widths),
        "ghost_persistence_s": max(ghost_persistence) if ghost_persistence else 0.0,
        "ghost_cell_count": len(ghost_persistence),
        "centroids": centroids,
        "classification": classification,
    }
