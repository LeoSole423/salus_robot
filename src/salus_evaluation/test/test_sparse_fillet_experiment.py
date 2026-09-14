import math

import pytest

from salus_evaluation.evaluation_runner import _experiment_geometry
from salus_evaluation.models import ExpectedTurn, GoalSpec, Pose2D


GOAL = GoalSpec(
    "single_corner_90", 8.0, 8.0, math.pi / 2.0, 90.0, ExpectedTurn.LEFT
)


def _local_poses(geometry):
    return tuple(
        ((point[0], point[1]), heading)
        for point, heading in geometry["poses"]
    )


def _assert_pose(actual, expected_xy, expected_yaw):
    assert actual[0] == pytest.approx(expected_xy)
    assert actual[1] == pytest.approx(expected_yaw)


def test_both_arms_use_three_poses_and_share_start_and_final():
    hard = _experiment_geometry(Pose2D(0.0, 0.0, 0.0), GOAL, "hard_vertex_current")
    sparse = _experiment_geometry(Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_fillet_r4")
    hard_poses, sparse_poses = _local_poses(hard), _local_poses(sparse)

    assert len(hard_poses) == len(sparse_poses) == 3
    _assert_pose(hard_poses[0], (4.0, 0.0), 0.0)
    _assert_pose(sparse_poses[0], (4.0, 0.0), 0.0)
    _assert_pose(hard_poses[-1], (8.0, 8.0), math.pi / 2.0)
    _assert_pose(sparse_poses[-1], (8.0, 8.0), math.pi / 2.0)
    _assert_pose(hard_poses[1], (8.0, 0.0), 0.0)
    _assert_pose(sparse_poses[1], (8.0, 4.0), math.pi / 2.0)


def test_sparse_arm_does_not_dispatch_internal_arc_samples():
    geometry = _experiment_geometry(Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_fillet_r4")
    dispatched = _local_poses(geometry)
    arc_points = geometry["fillet"]["arc_points"]
    assert len(dispatched) == 3
    assert all(point not in arc_points[1:-1] for point, _heading in dispatched)
    assert len(geometry["arm_reference"]) > len(dispatched)


def test_local_to_map_transforms_positions_and_yaws_for_nonzero_spawn_yaw():
    geometry = _experiment_geometry(Pose2D(10.0, -2.0, math.pi / 2.0), GOAL,
                                    "sparse_fillet_r4")
    poses = _local_poses(geometry)
    _assert_pose(poses[0], (10.0, 2.0), math.pi / 2.0)
    _assert_pose(poses[1], (6.0, 6.0), math.pi)
    _assert_pose(poses[-1], (2.0, 6.0), math.pi)


def test_arms_keep_the_same_common_and_nav2_reference_contract():
    hard = _experiment_geometry(Pose2D(0.0, 0.0, 0.0), GOAL, "hard_vertex_current")
    sparse = _experiment_geometry(Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_fillet_r4")
    assert hard["common_reference"] == sparse["common_reference"]
    assert hard["poses"][-1] == sparse["poses"][-1]
    assert hard["fillet"]["radius_m"] == pytest.approx(4.0)
