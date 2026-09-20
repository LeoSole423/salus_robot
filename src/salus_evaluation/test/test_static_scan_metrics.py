"""Pure tests for the obstacle-drag static scan geometry metrics."""

import math

import pytest

from salus_evaluation.models import Pose2D
from salus_evaluation.static_scan_metrics import (
    StaticObstacle, StaticScanMetrics, interpolate_pose, load_obstacle_geometry,
    ray_box_intersection, scan_static_error_metrics, summarize_scan_metrics,
)


FIXTURE = (
    __import__("pathlib").Path(__file__).parents[1].parent
    / "salus_simulation" / "config" / "obstacle_drag_geometry.yaml"
)


def test_ray_box_intersection_and_perfect_scan_have_zero_error() -> None:
    obstacle = StaticObstacle("box", 4.0, 0.0, 1.0, 1.0)
    ranges = []
    for index in range(181):
        angle = -math.pi / 2.0 + index * math.pi / 180.0
        hit = ray_box_intersection(0.0, 0.0, angle, obstacle)
        ranges.append(float("inf") if hit is None else hit)
    metrics = scan_static_error_metrics(
        ranges, -math.pi / 2.0, math.pi / 180.0, Pose2D(0.0, 0.0, 0.0), [obstacle],
        range_max_m=20.0,
    )
    assert metrics.sample_count > 0
    assert metrics.scan_static_error_rmse_m == pytest.approx(0.0)
    assert metrics.scan_static_error_p95_m == pytest.approx(0.0)


def test_scan_metrics_transform_robot_pose_and_measure_error() -> None:
    obstacle = StaticObstacle("box", 5.0, 2.0, 1.0, 1.0)
    pose = Pose2D(1.0, 1.0, math.pi / 2.0)
    ranges = []
    for index in range(181):
        angle = -math.pi / 2.0 + index * math.pi / 180.0
        hit = ray_box_intersection(pose.x_m, pose.y_m, pose.yaw_rad + angle, obstacle)
        ranges.append(float("inf") if hit is None else hit + 0.2)
    metrics = scan_static_error_metrics(
        ranges, -math.pi / 2.0, math.pi / 180.0, pose, [obstacle], range_max_m=20.0,
    )
    assert metrics.sample_count > 0
    assert metrics.scan_static_error_rmse_m == pytest.approx(0.2)
    assert metrics.scan_static_error_p95_m == pytest.approx(0.2)
    assert metrics.max_error_m == pytest.approx(0.2)
    assert metrics.worst_beam_indices


def test_interpolate_pose_uses_scan_timestamp_during_motion() -> None:
    samples = [
        (10.0, Pose2D(0.0, 0.0, 0.0)),
        (12.0, Pose2D(2.0, 0.0, math.pi / 2.0)),
    ]

    at_first_half = interpolate_pose(samples, 11.0)
    at_second = interpolate_pose(samples, 11.5)

    assert at_first_half == Pose2D(1.0, 0.0, math.pi / 4.0)
    assert at_second is not None
    assert at_second.x_m == pytest.approx(1.5)
    assert at_second.yaw_rad == pytest.approx(3.0 * math.pi / 8.0)
    assert interpolate_pose(samples, 9.0) is None
    assert interpolate_pose(samples, 13.0) is None


def test_motion_scans_use_pose_at_each_scan_timestamp() -> None:
    obstacle = StaticObstacle("box", 5.0, 0.0, 1.0, 1.0)
    poses = [
        (10.0, Pose2D(0.0, 0.0, 0.0)),
        (11.0, Pose2D(1.0, 0.0, 0.0)),
    ]

    def perfect_scan(pose: Pose2D) -> list[float]:
        ranges = []
        for index in range(181):
            angle = -math.pi / 2.0 + index * math.pi / 180.0
            hit = ray_box_intersection(pose.x_m, pose.y_m, angle, obstacle)
            ranges.append(float("inf") if hit is None else hit)
        return ranges

    first_scan_pose = interpolate_pose(poses, 10.0)
    second_scan_pose = interpolate_pose(poses, 11.0)
    assert first_scan_pose is not None and second_scan_pose is not None
    assert first_scan_pose != second_scan_pose
    first_metrics = scan_static_error_metrics(
        perfect_scan(first_scan_pose), -math.pi / 2.0, math.pi / 180.0,
        first_scan_pose, [obstacle], range_max_m=20.0,
    )
    second_metrics = scan_static_error_metrics(
        perfect_scan(second_scan_pose), -math.pi / 2.0, math.pi / 180.0,
        second_scan_pose, [obstacle], range_max_m=20.0,
    )

    assert first_metrics.scan_static_error_rmse_m == pytest.approx(0.0)
    assert second_metrics.scan_static_error_rmse_m == pytest.approx(0.0)


def test_obstacle_geometry_fixture_is_versioned_and_known() -> None:
    fixed_frame, obstacles = load_obstacle_geometry(FIXTURE)
    assert fixed_frame == "odom"
    assert [obstacle.name for obstacle in obstacles] == [
        "obstacle_near_post", "obstacle_mid_wall", "obstacle_far_box",
    ]
    assert [(obstacle.x_m, obstacle.y_m) for obstacle in obstacles] == [
        (2.5, 1.5), (7.5, -2.5), (14.0, 5.0),
    ]


def test_scan_summary_perfect_control_is_zero_and_schema_is_explicit() -> None:
    summary = summarize_scan_metrics([
        (12.0, StaticScanMetrics(3, 0.0, 0.0, 0.0)),
    ])

    assert summary == {
        "samples_per_scan": [3],
        "max_scan_static_error_rmse_m": 0.0,
        "max_scan_static_error_p95_m": 0.0,
        "max_beam_static_error_m": 0.0,
        "per_scan_metrics": [{
            "stamp_s": 12.0,
            "sample_count": 3,
            "rmse_m": 0.0,
            "p95_m": 0.0,
            "max_m": 0.0,
        }],
    }


def test_scan_summary_orders_scans_and_preserves_distinct_maxima() -> None:
    summary = summarize_scan_metrics([
        (20.0, StaticScanMetrics(2, 0.4, 0.6, 0.9)),
        (10.0, StaticScanMetrics(4, 0.1, 0.2, 0.3)),
    ])

    assert summary["samples_per_scan"] == [4, 2]
    assert summary["max_scan_static_error_rmse_m"] == pytest.approx(0.4)
    assert summary["max_scan_static_error_p95_m"] == pytest.approx(0.6)
    assert summary["max_beam_static_error_m"] == pytest.approx(0.9)
    assert [entry["stamp_s"] for entry in summary["per_scan_metrics"]] == [
        10.0, 20.0,
    ]
    assert summary["per_scan_metrics"][0]["max_m"] == pytest.approx(0.3)
    assert summary["per_scan_metrics"][1]["max_m"] == pytest.approx(0.9)
