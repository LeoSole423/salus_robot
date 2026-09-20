import math

import numpy as np

from salus_perception.scan_filters import (
    filter_cloud_points,
    obstacle_points,
    rotate_translate_point,
)


TOLERANCE = 0.20
MAX_RANGE = 20.0


def _legacy_filter(
    points,
    *,
    quaternion_xyzw,
    translation_xyz,
    ground_tolerance_m=TOLERANCE,
    max_range_m=MAX_RANGE,
):
    """Characterization implementation for rows emitted by ``read_points``."""
    transformed = [
        rotate_translate_point(
            tuple(float(value) for value in point),
            quaternion_xyzw=quaternion_xyzw,
            translation_xyz=translation_xyz,
        )
        for point in points
    ]
    return obstacle_points(
        transformed,
        ground_tolerance_m=ground_tolerance_m,
        max_range_m=max_range_m,
    )


def _assert_equivalent(points, *, quaternion_xyzw, translation_xyz, **kwargs):
    expected = np.asarray(
        _legacy_filter(
            points,
            quaternion_xyzw=quaternion_xyzw,
            translation_xyz=translation_xyz,
            **kwargs,
        ),
        dtype=np.float64,
    ).reshape((-1, 3))
    actual = filter_cloud_points(
        points,
        quaternion_xyzw=quaternion_xyzw,
        translation_xyz=translation_xyz,
        **kwargs,
    )
    np.testing.assert_array_equal(actual, expected)


def test_vectorized_filter_handles_empty_cloud() -> None:
    result = filter_cloud_points(
        [], quaternion_xyzw=(0.0, 0.0, 0.0, 1.0), translation_xyz=(0.0, 0.0, 0.0)
    )

    assert result.shape == (0, 3)
    assert result.dtype == np.float64


def test_vectorized_filter_matches_ground_and_range_boundaries() -> None:
    points = [
        (MAX_RANGE, 0.0, 0.0),
        (math.nextafter(MAX_RANGE, math.inf), 0.0, 0.0),
        (math.nextafter(MAX_RANGE, -math.inf), 0.0, 0.0),
        (1.0, 0.0, TOLERANCE),
        (1.0, 0.0, math.nextafter(TOLERANCE, math.inf)),
        (1.0, 0.0, math.nextafter(TOLERANCE, -math.inf)),
    ]

    _assert_equivalent(
        points,
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        translation_xyz=(0.0, 0.0, 0.0),
    )


def test_vectorized_filter_matches_nontrivial_transform() -> None:
    points = [
        (1.5, -2.0, 0.0),
        (-3.0, 0.4, 0.21),
        (8.0, 4.0, -0.5),
        (19.0, 0.0, 0.3),
    ]

    _assert_equivalent(
        points,
        quaternion_xyzw=(0.11, -0.23, 0.31, 0.87),
        translation_xyz=(0.7, -1.2, 0.15),
    )


def test_vectorized_filter_does_not_normalize_quaternion() -> None:
    points = [(1.0, 2.0, 0.3), (-2.0, 0.5, 0.0)]
    quaternion = (0.2, -0.4, 0.1, 1.7)
    translation = (0.4, 0.6, -0.2)

    _assert_equivalent(
        points,
        quaternion_xyzw=quaternion,
        translation_xyz=translation,
    )

    norm = math.sqrt(sum(item * item for item in quaternion))
    normalized = tuple(value / norm for value in quaternion)
    assert not np.array_equal(
        filter_cloud_points(
            points,
            quaternion_xyzw=quaternion,
            translation_xyz=translation,
        ),
        filter_cloud_points(
            points,
            quaternion_xyzw=normalized,
            translation_xyz=translation,
        ),
    )


def test_vectorized_filter_preserves_input_nans_and_nonfinite_outputs() -> None:
    points = [
        (float("nan"), 1.0, 1.0),
        (float("inf"), 0.0, 0.0),
        (1.0, float("-inf"), 0.0),
        (1.0, 0.0, float("inf")),
    ]

    _assert_equivalent(
        points,
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        translation_xyz=(0.0, 0.0, 0.0),
    )
    result = filter_cloud_points(
        points,
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        translation_xyz=(0.0, 0.0, 0.0),
    )
    assert len(result) == 4
    assert not np.isfinite(result).all()


def test_vectorized_filter_matches_math_hypot_at_non_axial_range_boundary() -> None:
    points = [(-18.927768950038683, 6.460616268898162, 0.0)]

    _assert_equivalent(
        points,
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        translation_xyz=(0.0, 0.0, 0.0),
    )
    assert math.hypot(*points[0][:2]) > MAX_RANGE
    result = filter_cloud_points(
        points,
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        translation_xyz=(0.0, 0.0, 0.0),
    )
    assert result.shape == (1, 3)


def test_vectorized_filter_preserves_obstacle_order() -> None:
    points = [
        (4.0, 0.0, 0.4),
        (1.0, 0.0, 0.0),
        (-5.0, 0.0, 0.8),
        (2.0, 0.0, 0.1),
        (0.0, 0.0, 0.9),
    ]

    result = filter_cloud_points(
        points,
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        translation_xyz=(0.0, 0.0, 0.0),
    )
    np.testing.assert_array_equal(result, np.asarray([points[0], points[2], points[4]]))


def test_vectorized_filter_matches_legacy_for_rs16_sized_cloud() -> None:
    generator = np.random.default_rng(274)
    points = generator.normal(size=(14_400, 3)).astype(np.float64)
    points[:, :2] *= 12.0
    points[:, 2] *= 1.5
    points[100, 0] = np.nan
    points[200, 1] = np.inf
    points[300, 2] = -np.inf

    _assert_equivalent(
        points,
        quaternion_xyzw=(0.11, -0.23, 0.31, 0.87),
        translation_xyz=(0.7, -1.2, 0.15),
    )
