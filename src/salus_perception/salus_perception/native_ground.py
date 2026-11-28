"""Adapter for the compiled radial classifier."""

import ctypes
import math

import numpy as np

from . import _radial_ground


_radial = ctypes.CDLL(_radial_ground.__file__).radial
_radial.argtypes = [ctypes.c_void_p] * 3 + [ctypes.c_int, ctypes.c_void_p] + [ctypes.c_double] * 6
_radial.restype = None


def classify(points, radii, indices, breaks, width, config):
    xyz = np.ascontiguousarray(points[indices], dtype=np.float64)
    distances = np.ascontiguousarray(radii[indices], dtype=np.float64)
    offsets = np.asarray([0, *breaks, len(indices)], dtype=np.int32)
    mask = np.empty(len(indices), dtype=np.uint8)
    _radial(
        xyz.ctypes.data, distances.ctypes.data, offsets.ctypes.data,
        len(offsets) - 1, mask.ctypes.data, width,
        math.tan(math.radians(config.global_slope_max_angle_deg)),
        math.radians(config.local_slope_max_angle_deg),
        config.split_points_distance_tolerance, config.split_height_distance,
        config.vehicle_wheel_base_m if config.use_virtual_ground_point else math.inf,
    )
    return points[indices[mask != 0]]
