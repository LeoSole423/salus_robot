"""Pure tests for the obstacle-drag static scan geometry metrics."""

import math

import pytest

from salus_evaluation.models import Pose2D
from salus_evaluation.static_scan_metrics import (
    StaticObstacle, load_obstacle_geometry, ray_box_intersection,
    scan_static_error_metrics,
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


def test_obstacle_geometry_fixture_is_versioned_and_known() -> None:
    fixed_frame, obstacles = load_obstacle_geometry(FIXTURE)
    assert fixed_frame == "odom"
    assert [obstacle.name for obstacle in obstacles] == [
        "obstacle_near_post", "obstacle_mid_wall", "obstacle_far_box",
    ]
    assert [(obstacle.x_m, obstacle.y_m) for obstacle in obstacles] == [
        (2.5, 1.5), (7.5, -2.5), (14.0, 5.0),
    ]
