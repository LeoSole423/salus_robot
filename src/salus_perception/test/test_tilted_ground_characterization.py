"""Offline characterization of the operator-reported tilt-dependent cloud."""

import math

from salus_perception.scan_filters import clean_ranges, filter_cloud_points


def _scene():
    # A fixed low, irregular grass patch to the robot's left, plus a real post.
    grass = [
        (6.0 + 0.25 * ix, 1.5 + 0.1 * iy, 0.08 + 0.015 * ((ix + 2 * iy) % 5))
        for ix in range(25) for iy in range(26)
    ]
    post = [(2.5, -0.1 + 0.025 * index, 0.75) for index in range(9)]
    return grass, post


def _project_and_clean(points):
    """Nearest-range binning with the real projection and speckle parameters."""
    angle_min, angle_max, increment = -math.pi / 2, math.pi / 2, 0.00872665
    ranges = [math.inf] * (int((angle_max - angle_min) / increment) + 1)
    for x, y, z in points:
        radial = math.hypot(x, y)
        if not (-0.1 <= z <= 1.6 and 0.4 <= radial <= 20.0):
            continue
        index = int((math.atan2(y, x) - angle_min) / increment)
        if 0 <= index < len(ranges):
            ranges[index] = min(ranges[index], radial)
    return clean_ranges(ranges)


def test_fixed_low_grass_appears_as_left_scan_occupancy_only_when_rolled():
    grass, post = _scene()
    upright = filter_cloud_points(
        grass + post, quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        translation_xyz=(0.0, 0.0, 0.0),
    )
    roll = math.radians(8.0)
    tilted = filter_cloud_points(
        grass + post,
        quaternion_xyzw=(math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2)),
        translation_xyz=(0.0, 0.0, 0.0),
    )
    straightened = filter_cloud_points(
        grass + post, quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        translation_xyz=(0.0, 0.0, 0.0),
    )

    upright_ranges = _project_and_clean(upright)
    tilted_ranges = _project_and_clean(tilted)
    straightened_ranges = _project_and_clean(straightened)
    left_sector = lambda ranges: sum(
        math.isfinite(value) for index, value in enumerate(ranges)
        if math.radians(10) <= -math.pi / 2 + index * 0.00872665 <= math.radians(40)
    )

    assert len(upright) == len(post)
    assert len(tilted) > len(post) + 500
    assert len(straightened) == len(post)
    assert left_sector(upright_ranges) == 0
    assert left_sector(tilted_ranges) >= 30
    assert left_sector(straightened_ranges) == 0
    # The real post remains detectable in every pose.
    assert all(any(math.isfinite(value) and value < 3.0 for value in ranges)
               for ranges in (upright_ranges, tilted_ranges, straightened_ranges))
