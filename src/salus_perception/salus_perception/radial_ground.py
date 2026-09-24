"""Pure radial ground classification for a leveled robot frame.

Behavioral reference: ROS2_SALUS scan_ground_filter (868b2fc, b3cdc58).
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class RadialGroundConfig:
    global_slope_max_angle_deg: float = 10.0
    local_slope_max_angle_deg: float = 13.0
    radial_divider_angle_deg: float = 1.0
    split_points_distance_tolerance: float = 0.20
    use_virtual_ground_point: bool = True
    split_height_distance: float = 0.20
    vehicle_wheel_base_m: float = 0.90
    range_max: float = 20.0


def non_ground_points(points: np.ndarray, config: RadialGroundConfig) -> np.ndarray:
    """Return obstacle points; input is XYZ in base_footprint coordinates."""
    if len(points) == 0:
        return points
    valid = np.isfinite(points).all(axis=1)
    radii = np.hypot(points[:, 0], points[:, 1])
    selected = np.flatnonzero(valid & (radii <= config.range_max))
    invalid = points[~valid]
    if len(selected) == 0:
        return invalid

    azimuth = np.mod(np.arctan2(points[selected, 0], points[selected, 1]), 2 * math.pi)
    width = math.radians(config.radial_divider_angle_deg)
    sectors = np.floor(azimuth / width).astype(np.int32)
    order = np.lexsort((radii[selected], sectors))
    indices = selected[order]
    sectors = sectors[order]
    breaks = np.flatnonzero(np.diff(sectors)) + 1
    obstacles = []
    local_limit = math.radians(config.local_slope_max_angle_deg)
    global_limit = math.tan(math.radians(config.global_slope_max_angle_deg))

    for ray in np.split(indices, breaks):
        ground_r = ground_z = ground_slope = 0.0
        ground_sum_r = ground_sum_z = ground_count = 0.0
        object_sum_z = object_count = 0.0
        last_label = 'initial'
        last_point = None
        for offset, index in enumerate(ray):
            x, y, z = (float(v) for v in points[index])
            radius = float(radii[index])
            if offset == 0:
                ground_r = config.vehicle_wheel_base_m if config.use_virtual_ground_point and x > config.vehicle_wheel_base_m else 0.0
                origin = (ground_r, 0.0, 0.0)
            else:
                origin = last_point
            distance = math.dist((x, y, z), origin)
            close = distance < radius * width + config.split_points_distance_tolerance
            delta_z = z - ground_z
            delta_r = radius - ground_r
            if close and ground_count:
                delta_z = z - ground_sum_z / ground_count
                delta_r = radius - ground_sum_r / ground_count

            if radius > 0 and z / radius > global_limit:
                label = 'object'
            elif last_label == 'ground' and close and abs(delta_z) < config.split_height_distance:
                label = 'follow'
            else:
                slope = math.atan2(delta_z, delta_r)
                label = 'object' if slope - ground_slope > local_limit else 'ground'

            if label == 'ground':
                ground_sum_r = ground_sum_z = ground_count = 0.0
                object_sum_z = object_count = 0.0
            if label == 'object':
                obstacles.append(index)
                object_sum_z += z
                object_count += 1
            else:
                ground_r, ground_z = radius, z
                ground_sum_r += radius
                ground_sum_z += z
                ground_count += 1
                ground_slope = math.atan2(ground_sum_z / ground_count, ground_sum_r / ground_count)
            last_label = 'ground' if label == 'follow' else label
            last_point = (x, y, z)

    result = points[np.asarray(obstacles, dtype=np.int64)]
    return np.concatenate((result, invalid)) if len(invalid) else result
