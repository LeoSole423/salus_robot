"""Pure deterministic LiDAR filters shared by ROS adapters and tests."""
from __future__ import annotations
import math
from typing import Iterable, Sequence

import numpy as np


Point = tuple[float, float, float]


def rotate_translate_point(
    point: Point,
    *,
    quaternion_xyzw: Sequence[float],
    translation_xyz: Sequence[float],
) -> Point:
    """Apply the existing quaternion-plus-translation formula to one point.

    The quaternion is deliberately not normalized.  Keeping this scalar
    reference in the pure module makes the vectorized implementation easy to
    characterize without importing ROS messages or starting a node.
    """
    x, y, z = point
    qx, qy, qz, qw = quaternion_xyzw
    tx, ty, tz = translation_xyz
    ix = qw * x + qy * z - qz * y
    iy = qw * y + qz * x - qx * z
    iz = qw * z + qx * y - qy * x
    iw = -qx * x - qy * y - qz * z
    return (
        ix * qw + iw * -qx + iy * -qz - iz * -qy + tx,
        iy * qw + iw * -qy + iz * -qx - ix * -qz + ty,
        iz * qw + iw * -qz + ix * -qy - iy * -qx + tz,
    )


def rotate_translate_points(
    points: Iterable[Point] | np.ndarray,
    *,
    quaternion_xyzw: Sequence[float],
    translation_xyz: Sequence[float],
) -> np.ndarray:
    """Vectorize :func:`rotate_translate_point` while preserving point order."""
    values = _as_points_array(points)
    if not len(values):
        return values

    x, y, z = values[:, 0], values[:, 1], values[:, 2]
    qx, qy, qz, qw = (float(value) for value in quaternion_xyzw)
    tx, ty, tz = (float(value) for value in translation_xyz)

    # Keep the operation grouping identical to the historical scalar formula.
    with np.errstate(invalid="ignore", over="ignore"):
        ix = qw * x + qy * z - qz * y
        iy = qw * y + qz * x - qx * z
        iz = qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z
        return np.column_stack(
            (
                ix * qw + iw * -qx + iy * -qz - iz * -qy + tx,
                iy * qw + iw * -qy + iz * -qx - ix * -qz + ty,
                iz * qw + iw * -qz + ix * -qy - iy * -qx + tz,
            )
        )


def obstacle_points_vectorized(
    points: Iterable[Point] | np.ndarray,
    *,
    ground_tolerance_m: float = 0.20,
    max_range_m: float = 20.0,
) -> np.ndarray:
    """Return obstacle rows using the same finite/range/height policy.

    Non-finite rows are intentionally retained as obstacles.  This mirrors
    ``obstacle_points`` and is important for post-transform ``Inf`` and NaN
    values.  PointCloud2 input-NaN filtering belongs to the ROS adapter's
    ``read_points(..., skip_nans=True)`` call.
    """
    values = _as_points_array(points)
    if not len(values):
        return values

    with np.errstate(invalid="ignore", over="ignore"):
        finite = np.isfinite(values).all(axis=1)
        radial = np.hypot(values[:, 0], values[:, 1])
        range_ok = radial <= max_range_m
        # NumPy and math.hypot can differ by one ULP at the boundary.  The
        # normal path remains vectorized; only a tiny boundary band falls back
        # to the legacy scalar operation for exact classification parity.
        boundary_ulp = abs(float(np.spacing(max_range_m)))
        near_limit = (
            np.isfinite(radial)
            & (np.abs(radial - max_range_m) <= 4.0 * boundary_ulp)
        )
        for index in np.flatnonzero(near_limit):
            range_ok[index] = math.hypot(
                float(values[index, 0]), float(values[index, 1])
            ) <= max_range_m
        ground = finite & range_ok & (values[:, 2] <= ground_tolerance_m)
    return values[~ground]


def filter_cloud_points(
    points: Iterable[Point] | np.ndarray,
    *,
    quaternion_xyzw: Sequence[float],
    translation_xyz: Sequence[float],
    ground_tolerance_m: float = 0.20,
    max_range_m: float = 20.0,
) -> np.ndarray:
    """Transform and filter a cloud without ROS dependencies.

    Input rows are transformed and classified without filtering.  The ROS
    adapter owns PointCloud2 NaN handling through
    ``read_points(..., skip_nans=True)``; non-finite rows reaching this core
    are retained as obstacles after transformation.
    """
    values = _as_points_array(points)
    transformed = rotate_translate_points(
        values,
        quaternion_xyzw=quaternion_xyzw,
        translation_xyz=translation_xyz,
    )
    return obstacle_points_vectorized(
        transformed,
        ground_tolerance_m=ground_tolerance_m,
        max_range_m=max_range_m,
    )


def _as_points_array(points: Iterable[Point] | np.ndarray) -> np.ndarray:
    """Normalize point input to a two-dimensional float64 array."""
    raw_points = list(points) if not isinstance(points, np.ndarray) else points
    values = np.asarray(raw_points, dtype=np.float64)
    if values.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    return values

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
