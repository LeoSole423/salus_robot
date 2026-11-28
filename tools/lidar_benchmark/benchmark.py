#!/usr/bin/env python3
"""Offline benchmark from recorded PointCloud2 CDR to output PointCloud2 CDR."""
import argparse
from array import array
import collections
import ctypes
import json
import math
from pathlib import Path
import sqlite3
import time

import cupy as cp
import numpy as np
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from salus_perception.radial_ground import RadialGroundConfig, non_ground_points
from salus_perception.scan_filters import rotate_translate_points
from salus_perception.scan_ground_filter import cloud_xyz_points

from resident_gpu import ResidentGroundFilter


CONFIG = RadialGroundConfig()
WIDTH = math.radians(CONFIG.radial_divider_angle_deg)
LIMITS = (
    WIDTH, math.tan(math.radians(CONFIG.global_slope_max_angle_deg)),
    math.radians(CONFIG.local_slope_max_angle_deg),
    CONFIG.split_points_distance_tolerance, CONFIG.split_height_distance, CONFIG.vehicle_wheel_base_m,
)
FIELDS = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
          for offset, name in zip((0, 4, 8), "xyz")]


def sort_sectors(points):
    valid = np.isfinite(points).all(axis=1)
    radii = np.hypot(points[:, 0], points[:, 1])
    selected = np.flatnonzero(valid & (radii <= CONFIG.range_max))
    azimuth = np.mod(np.arctan2(points[selected, 0], points[selected, 1]), 2 * math.pi)
    sectors = np.floor(azimuth / WIDTH).astype(np.int32)
    order = np.lexsort((radii[selected], sectors))
    indices = selected[order]
    offsets = np.array([0, *(np.flatnonzero(np.diff(sectors[order])) + 1), len(indices)], dtype=np.int32)
    return points[indices], radii[indices], offsets, points[~valid]


class SectorBackends:
    def __init__(self, source, native_library):
        self.gpu = cp.RawKernel(source.read_text(), "radial", options=("--fmad=false",))
        self.gpu.compile()
        self.resident = ResidentGroundFilter(self.gpu, LIMITS, CONFIG.range_max)
        self.cpu = ctypes.CDLL(str(native_library)).radial
        self.cpu.argtypes = [ctypes.c_void_p] * 3 + [ctypes.c_int, ctypes.c_void_p] + [ctypes.c_double] * 6
        self.cpu.restype = None

    def classify(self, points, use_gpu):
        xyz, radii, offsets, invalid = sort_sectors(points)
        sectors = len(offsets) - 1
        if use_gpu:
            mask_gpu = cp.empty(len(xyz), dtype=cp.uint8)
            self.gpu(((sectors + 127) // 128,), (128,),
                     (cp.asarray(xyz), cp.asarray(radii), cp.asarray(offsets), np.int32(sectors),
                      mask_gpu, *(np.float64(x) for x in LIMITS)))
            mask = cp.asnumpy(mask_gpu)
        else:
            mask = np.empty(len(xyz), dtype=np.uint8)
            self.cpu(xyz.ctypes.data, radii.ctypes.data, offsets.ctypes.data, sectors, mask.ctypes.data, *LIMITS)
        return np.concatenate((xyz[mask != 0], invalid))


def fast_cloud(header, points):
    data = array("B")
    data.frombytes(points.astype("<f4", copy=False).tobytes())
    return PointCloud2(header=header, height=1, width=len(points), fields=FIELDS,
                       is_bigendian=False, point_step=12, row_step=12 * len(points),
                       data=data, is_dense=False)


def frames_from_bag(path, size):
    with sqlite3.connect(f"{path.resolve().as_uri()}?mode=ro&immutable=1", uri=True) as db:
        ids = dict(db.execute("SELECT name,id FROM topics"))
        frame, = db.execute("SELECT data FROM messages WHERE topic_id=? LIMIT 1", (ids["/scan_3d"],)).fetchone()
        frame_id = deserialize_message(frame, PointCloud2).header.frame_id
        tf = Buffer()
        for blob, in db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (ids["/tf_static"],)):
            for transform in deserialize_message(blob, TFMessage).transforms:
                tf.set_transform_static(transform, "offline")
        transform = tf.lookup_transform("base_footprint", frame_id, Time()).transform
        count, = db.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?", (ids["/scan_3d"],)).fetchone()
        targets = set(np.linspace(0, count - 1, min(size, count), dtype=int))
        for i, (blob,) in enumerate(db.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp", (ids["/scan_3d"],))):
            if i in targets:
                yield transform, blob


def filter_frame(blob, transform, backends, method):
    start = time.perf_counter_ns()
    cpu_start = time.process_time_ns()
    msg = deserialize_message(blob, PointCloud2)
    q = transform.rotation
    t = transform.translation
    source = cloud_xyz_points(msg)
    if method in {"cuda_resident", "cuda_boundary"}:
        obstacles = backends.resident.classify(source, q, t, correct_boundaries=method == "cuda_boundary")
    else:
        points = rotate_translate_points(source, quaternion_xyzw=(q.x, q.y, q.z, q.w),
                                         translation_xyz=(t.x, t.y, t.z))
        if method in ("actual", "numpy_pack"):
            obstacles = non_ground_points(points, CONFIG)
        else:
            obstacles = backends.classify(points, method == "cuda")
    msg.header.frame_id = "base_footprint"
    out = (point_cloud2.create_cloud_xyz32(msg.header, obstacles.tolist()) if method == "actual"
           else fast_cloud(msg.header, obstacles))
    packed = serialize_message(out)
    cpu_elapsed = (time.process_time_ns() - cpu_start) / 1e6
    elapsed = (time.perf_counter_ns() - start) / 1e6
    return packed, elapsed, len(source), len(obstacles), cpu_elapsed


def project_scan(points):
    """Reproduce the min-range aggregation for the real projection profile."""
    angle_min, angle_max, step = np.float32([-1.5707963, 1.5707963, .00872665])
    rays = np.full(math.ceil((angle_max - angle_min) / step), np.inf, dtype=np.float32)
    x, y, z = points.astype(np.float64).T
    ranges = np.hypot(x, y)
    angles = np.arctan2(y, x)
    valid = (np.isfinite(points).all(axis=1) & (z >= -.1) & (z <= 1.6)
             & (ranges >= .4) & (ranges <= 20.) & (angles >= angle_min) & (angles <= angle_max))
    ids = ((angles[valid] - angle_min) / step).astype(np.int32)
    within = ids < len(rays)
    np.minimum.at(rays, ids[within], ranges[valid][within].astype(np.float32))
    return rays


def inspect_content(blob, transform, reference, candidate, backend):
    original = cloud_xyz_points(deserialize_message(blob, PointCloud2))
    q, t = transform.rotation, transform.translation
    xyz = rotate_translate_points(original, quaternion_xyzw=(q.x, q.y, q.z, q.w),
                                  translation_xyz=(t.x, t.y, t.z))
    device_xyz, device_radii, device_sectors = [cp.asnumpy(a) for a in backend.device_geometry(original, q, t)[:3]]
    radii = np.hypot(xyz[:, 0], xyz[:, 1])
    valid = np.isfinite(xyz).all(axis=1) & (radii <= CONFIG.range_max)
    sectors = np.floor(np.mod(np.arctan2(xyz[valid, 0], xyz[valid, 1]), math.tau) / WIDTH)
    ref_points = np.frombuffer(reference.data, dtype="<f4").reshape(-1, 3)
    candidate_points = np.frombuffer(candidate.data, dtype="<f4").reshape(-1, 3)
    rs, cs = project_scan(ref_points), project_scan(candidate_points)
    finite_both = np.isfinite(rs) & np.isfinite(cs)
    reference_counts = collections.Counter(row.tobytes() for row in ref_points)
    candidate_counts = collections.Counter(row.tobytes() for row in candidate_points)
    return {
        "input_transformed_xyz_differences": int(np.count_nonzero(xyz != device_xyz)),
        "input_radius_differences_in_range": int(np.count_nonzero(radii[valid] != device_radii[valid])),
        "input_sector_differences_in_range": int(np.count_nonzero(sectors != device_sectors[valid])),
        "missing_obstacle_points": sum((reference_counts - candidate_counts).values()),
        "extra_obstacle_points": sum((candidate_counts - reference_counts).values()),
        "projected_scan_different_bins": int(np.count_nonzero(rs != cs)),
        "projected_scan_lost_finite_bins": int(np.count_nonzero(np.isfinite(rs) & ~np.isfinite(cs))),
        "projected_scan_max_finite_range_delta_m": float(np.max(np.abs(rs[finite_both] - cs[finite_both]), initial=0)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path)
    parser.add_argument("--frames-per-bag", type=int, default=80)
    parser.add_argument("--pace", type=float, default=0, help="Seconds between measured frames")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--native-library", type=Path, required=True)
    parser.add_argument("--methods", nargs="+", choices=("numpy_pack", "cpu_native", "cuda", "cuda_resident", "cuda_boundary"),
                        default=("numpy_pack", "cpu_native", "cuda", "cuda_resident", "cuda_boundary"))
    args = parser.parse_args()
    methods = ("actual", *args.methods)
    backends = SectorBackends(Path(__file__).with_name("radial_probe.cu"), args.native_library)
    counts = collections.Counter()
    cdr_differences = collections.Counter()
    differences = {}
    order_only = collections.Counter()
    content_mismatch = collections.Counter()
    content_details = []
    timings = {method: [] for method in methods}
    cpu_timings = {method: [] for method in methods}
    observed = []
    for bag in args.bags:
        for i, (transform, blob) in enumerate(frames_from_bag(bag, args.frames_per_bag)):
            if i == 0:
                for method in methods:
                    filter_frame(blob, transform, backends, method)
            order = methods[i % len(methods):] + methods[:i % len(methods)]
            values = {method: filter_frame(blob, transform, backends, method) for method in order}
            observed.append(values["actual"][2:4])
            if args.pace:
                time.sleep(args.pace)
            for method in methods:
                timings[method].append(values[method][1])
                cpu_timings[method].append(values[method][4])
                if values[method][0] != values["actual"][0]:
                    cdr_differences[method] += 1
                    reference = deserialize_message(values["actual"][0], PointCloud2)
                    candidate = deserialize_message(values[method][0], PointCloud2)
                    if reference != candidate:
                        counts[method] += 1
                        ref_xyz = np.frombuffer(reference.data, dtype="V12")
                        candidate_xyz = np.frombuffer(candidate.data, dtype="V12")
                        same_points = np.array_equal(np.sort(ref_xyz), np.sort(candidate_xyz))
                        same_layout = (
                            reference.header == candidate.header and reference.fields == candidate.fields
                            and (reference.height, reference.width, reference.is_bigendian,
                                 reference.point_step, reference.row_step, reference.is_dense)
                            == (candidate.height, candidate.width, candidate.is_bigendian,
                                candidate.point_step, candidate.row_step, candidate.is_dense)
                        )
                        if same_points and same_layout:
                            order_only[method] += 1
                        else:
                            content_mismatch[method] += 1
                            if method in {"cuda_resident", "cuda_boundary"}:
                                content_details.append({
                                    "dataset": bag.parent.parent.name, "frame": i, "method": method,
                                    **inspect_content(blob, transform, reference, candidate, backends.resident),
                                })
                        differences.setdefault(method, {
                            "bag": bag.name, "dataset": bag.parent.parent.name, "frame": i,
                            "reference_points": reference.width,
                            "candidate_points": candidate.width,
                            "same_obstacle_multiset": same_points,
                            "same_layout": same_layout,
                        })
    actual_median = float(np.median(timings["actual"]))
    summary = {
        "gpu": cp.cuda.runtime.getDeviceProperties(0)["name"].decode(),
        "frames": len(observed),
        "bags": [str(bag) for bag in args.bags],
        "input_points_total": sum(points for points, _ in observed),
        "input_points_range": [min(points for points, _ in observed), max(points for points, _ in observed)],
        "obstacle_points_total": sum(obstacles for _, obstacles in observed),
        "first_semantic_difference": differences,
        "content_difference_details": content_details,
        "methods": {
            method: {
                "median_ms": float(np.median(times)),
                "host_cpu_median_ms": float(np.median(cpu_timings[method])),
                "p95_ms": float(np.percentile(times, 95)),
                "max_ms": max(times),
                "median_speedup": actual_median / float(np.median(times)),
                "different_output_frames": counts[method],
                "different_order_only_frames": order_only[method],
                "different_content_frames": content_mismatch[method],
                "different_cdr_byte_frames": cdr_differences[method],
            }
            for method, times in timings.items()
        },
    }
    args.output.write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    if content_mismatch:
        raise SystemExit("Candidate changed the obstacle multiset or PointCloud2 layout.")


if __name__ == "__main__":
    main()
