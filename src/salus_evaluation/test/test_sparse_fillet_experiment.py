import math

import pytest

from salus_evaluation.evaluation_runner import (
    _boundary_transition_metrics,
    _experiment_geometry,
)
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


def test_sparse_single_3_uses_exact_e_x_f_constraints():
    geometry = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_single_3"
    )
    requests = geometry["request_poses"]
    assert len(requests) == 1
    assert len(requests[0]) == 3
    _assert_pose(requests[0][0], (4.0, 0.0), 0.0)
    _assert_pose(requests[0][1], (8.0, 4.0), math.pi / 2.0)
    _assert_pose(requests[0][2], (8.0, 8.0), math.pi / 2.0)
    assert (8.0, 0.0) not in [point for point, _yaw in requests[0]]


def test_boundary_exit_partitions_same_sparse_constraints_without_vertex():
    single = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_single_3"
    )
    boundary = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_boundary_exit"
    )
    assert len(boundary["request_poses"]) == 2
    assert [item for request in boundary["request_poses"] for item in request] == list(
        single["request_poses"][0]
    )
    assert all(
        point != (8.0, 0.0)
        for request in boundary["request_poses"]
        for point, _yaw in request
    )


def test_single_4_and_midarc_boundary_share_exact_four_constraints():
    single = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_single_4"
    )
    boundary = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_boundary_midarc"
    )
    assert len(single["request_poses"][0]) == 4
    assert [item for request in boundary["request_poses"] for item in request] == list(
        single["request_poses"][0]
    )
    assert single["midpoint"][0] == pytest.approx((6.8284271247, 1.1715728753))
    assert single["midpoint"][1] == pytest.approx(math.pi / 4.0)


@pytest.mark.parametrize("variant, expected_counts", [
    ("sparse_single_3", (3,)),
    ("sparse_boundary_exit", (2, 1)),
    ("sparse_single_4", (4,)),
    ("sparse_boundary_midarc", (2, 2)),
])
def test_sparse_request_partition_has_exact_pose_counts_and_order(variant, expected_counts):
    geometry = _experiment_geometry(Pose2D(0.0, 0.0, 0.0), GOAL, variant)
    assert tuple(len(request) for request in geometry["request_poses"]) == expected_counts
    assert all(
        point != (8.0, 0.0)
        for request in geometry["request_poses"]
        for point, _yaw in request
    )


def test_boundary_provenance_keeps_plans_separate_and_records_robot_dispatch_pose():
    geometry = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "sparse_boundary_exit"
    )
    requests = [
        {
            "request_index": 0,
            "dispatch_stamp_s": 1.0,
            "result_stamp_s": 2.0,
            "poses": tuple(
                {"x_m": point[0], "y_m": point[1], "yaw_rad": yaw}
                for point, yaw in geometry["request_poses"][0]
            ),
            "robot_pose_at_dispatch": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        },
        {
            "request_index": 1,
            "dispatch_stamp_s": 2.1,
            "result_stamp_s": None,
            "poses": tuple(
                {"x_m": point[0], "y_m": point[1], "yaw_rad": yaw}
                for point, yaw in geometry["request_poses"][1]
            ),
            "robot_pose_at_dispatch": {"x_m": 8.0, "y_m": 4.0, "yaw_rad": math.pi / 2},
        },
    ]
    plans = [
        {"request_index": 0, "points": tuple(
            (Pose2D(4.0, 0.0, 0.0), Pose2D(8.0, 4.0, math.pi / 2))
        )},
        {"request_index": 1, "points": tuple(
            (Pose2D(8.0, 4.0, math.pi / 2), Pose2D(8.0, 8.0, math.pi / 2))
        )},
    ]
    result = _boundary_transition_metrics(
        requests, plans, geometry["arm_reference"]
    )
    assert result["available"] is True
    assert result["plan_count_A"] == result["plan_count_B"] == 1
    assert result["robot_yaw_at_dispatch_B_rad"] == pytest.approx(math.pi / 2)
    assert result["terminal_A_to_dispatch_B_s"] == pytest.approx(0.1)
