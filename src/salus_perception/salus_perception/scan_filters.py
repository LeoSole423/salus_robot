"""Pure deterministic LiDAR filters shared by ROS adapters and tests."""
from __future__ import annotations
import math
from typing import Iterable

def is_ground_point(x: float, y: float, z: float, *, ground_tolerance_m: float = 0.20, max_range_m: float = 20.0) -> bool:
    return all(math.isfinite(v) for v in (x, y, z)) and math.hypot(x, y) <= max_range_m and z <= ground_tolerance_m

def obstacle_points(points: Iterable[tuple[float, float, float]], *, ground_tolerance_m: float = 0.20, max_range_m: float = 20.0) -> list[tuple[float, float, float]]:
    return [point for point in points if not is_ground_point(*point, ground_tolerance_m=ground_tolerance_m, max_range_m=max_range_m)]

def clean_ranges(ranges: Iterable[float], *, range_min: float = 0.4, range_max: float = 20.0, speckle_window: int = 2, speckle_max_range: float = 12.0, max_deviation_m: float = 0.30) -> list[float]:
    values = [v if math.isfinite(v) and range_min <= v <= range_max else math.inf for v in ranges]
    result = list(values)
    for index, value in enumerate(values):
        if not math.isfinite(value) or value > speckle_max_range: continue
        neighbours = [other for pos, other in enumerate(values[max(0,index-speckle_window):index+speckle_window+1], max(0,index-speckle_window)) if pos != index and math.isfinite(other) and abs(other-value) <= max_deviation_m]
        if not neighbours: result[index] = math.inf
    return result
