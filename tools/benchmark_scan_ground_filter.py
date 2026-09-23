#!/usr/bin/env python3
"""Manual PC benchmark for the scalar and vectorized ground filter paths.

Example (from the repository root after sourcing the ROS workspace)::

    python3 tools/benchmark_scan_ground_filter.py --points 14400 \
        --warmup 5 --repetitions 30

The script reports median and p95 only; timing is intentionally not a CI
assertion.  ``end-to-end`` includes the point-array preparation and output
materialization performed around the pure core by the ROS adapter.  It does
not start ROS or serialize a PointCloud2 message, so the Jetson gate remains
open.
"""
from __future__ import annotations

import argparse
from time import perf_counter

import numpy as np

from salus_perception.scan_filters import (
    filter_cloud_points,
    obstacle_points,
    rotate_translate_point,
)


QUATERNION = (0.11, -0.23, 0.31, 0.87)
TRANSLATION = (0.7, -1.2, 0.15)
TOLERANCE = 0.20
MAX_RANGE = 20.0


def make_cloud(size: int, seed: int) -> np.ndarray:
    generator = np.random.default_rng(seed)
    points = generator.normal(size=(size, 3)).astype(np.float64)
    points[:, :2] *= 12.0
    points[:, 2] *= 1.5
    # Keep a deterministic mix of ground, curb and distant obstacle points.
    if size:
        points[::11, 2] = 0.0
        points[::17, 2] = 0.35
        points[::19, 0] = 22.0
    return points


def legacy_filter(points: np.ndarray) -> list[tuple[float, float, float]]:
    transformed = [
        rotate_translate_point(
            tuple(float(value) for value in point),
            quaternion_xyzw=QUATERNION,
            translation_xyz=TRANSLATION,
        )
        for point in points
    ]
    return obstacle_points(
        transformed,
        ground_tolerance_m=TOLERANCE,
        max_range_m=MAX_RANGE,
    )


def vectorized_filter(points: np.ndarray) -> np.ndarray:
    return filter_cloud_points(
        points,
        quaternion_xyzw=QUATERNION,
        translation_xyz=TRANSLATION,
        ground_tolerance_m=TOLERANCE,
        max_range_m=MAX_RANGE,
    )


def legacy_end_to_end(points: np.ndarray) -> list[tuple[float, float, float]]:
    # Model the adapter's read_points/list boundary before the scalar core.
    return list(legacy_filter(np.asarray(list(points), dtype=np.float64)))


def vectorized_end_to_end(points: np.ndarray) -> list[list[float]]:
    # Model the adapter's array boundary and create_cloud_xyz32 input list.
    return vectorized_filter(np.asarray(list(points), dtype=np.float64)).tolist()


def timed(function, points: np.ndarray, warmup: int, repetitions: int) -> np.ndarray:
    for _ in range(warmup):
        function(points)
    samples = []
    for _ in range(repetitions):
        started = perf_counter()
        function(points)
        samples.append((perf_counter() - started) * 1000.0)
    return np.asarray(samples, dtype=np.float64)


def report(label: str, samples: np.ndarray) -> None:
    print(
        f"{label:22s} median={np.median(samples):9.3f} ms "
        f"p95={np.percentile(samples, 95):9.3f} ms"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points", type=int, default=14_400)
    parser.add_argument("--seed", type=int, default=274)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--repetitions", type=int, default=30)
    args = parser.parse_args()
    if args.points < 0 or args.warmup < 0 or args.repetitions < 1:
        parser.error("points and warmup must be non-negative; repetitions must be positive")

    points = make_cloud(args.points, args.seed)
    legacy_core = timed(legacy_filter, points, args.warmup, args.repetitions)
    vectorized_core = timed(vectorized_filter, points, args.warmup, args.repetitions)
    legacy_e2e = timed(legacy_end_to_end, points, args.warmup, args.repetitions)
    vectorized_e2e = timed(vectorized_end_to_end, points, args.warmup, args.repetitions)

    print(f"points={args.points} warmup={args.warmup} repetitions={args.repetitions}")
    report("legacy core", legacy_core)
    report("vectorized core", vectorized_core)
    report("legacy end-to-end", legacy_e2e)
    report("vectorized end-to-end", vectorized_e2e)
    print(f"core median speedup={np.median(legacy_core) / np.median(vectorized_core):.2f}x")
    print(f"end-to-end median speedup={np.median(legacy_e2e) / np.median(vectorized_e2e):.2f}x")
    print("criterion: observed median speedup >= 2x at 14400 points; no timing assertion is made")


if __name__ == "__main__":
    main()
