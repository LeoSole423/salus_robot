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


def test_track3_fixture_derives_the_frozen_points_and_fillet_contract():
    current = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "track3_current_boundary"
    )
    points = current["logical_points"]
    assert points["P0"]["x_m"] == pytest.approx(1.5)
    assert points["P0"]["y_m"] == pytest.approx(0.0)
    assert points["P1"]["x_m"] == pytest.approx(5.5)
    assert points["P1"]["y_m"] == pytest.approx(1.0717967697)
    assert points["P2"]["x_m"] == pytest.approx(8.4282032303)
    assert points["P2"]["y_m"] == pytest.approx(4.0)
    assert points["E1"]["x_m"] == pytest.approx(4.4647238196)
    assert points["E1"]["y_m"] == pytest.approx(0.7943953532)
    assert points["X1"]["x_m"] == pytest.approx(6.2578747639)
    assert points["X1"]["y_m"] == pytest.approx(1.8296715337)
    assert points["S1"]["x_m"] == pytest.approx(6.2071067812)
    assert points["S1"]["y_m"] == pytest.approx(1.7789035509)
    assert current["fillet"]["radius_m"] == pytest.approx(4.0)
    assert current["track3_nominal_radius_m"] == pytest.approx(8.0)
    assert current["planner_minimum_turning_radius_m"] == pytest.approx(4.0)


def test_track3_matched_requests_have_only_the_two_contractual_requests():
    current = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "track3_current_boundary"
    )
    sparse = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "track3_sparse_exit"
    )
    assert tuple(len(request) for request in current["request_poses"]) == (2, 2)
    assert tuple(len(request) for request in sparse["request_poses"]) == (2, 1)
    _assert_pose(current["request_poses"][0][0], (4.4647238196, 0.7943953532), math.radians(15.0))
    _assert_pose(current["request_poses"][0][1], (5.5, 1.0717967697), math.radians(15.0))
    _assert_pose(current["request_poses"][1][0], (6.2071067812, 1.7789035509), math.radians(45.0))
    _assert_pose(current["request_poses"][1][1], (8.4282032303, 4.0), math.radians(45.0))
    assert sparse["request_poses"][0][0] == current["request_poses"][0][0]
    _assert_pose(sparse["request_poses"][0][1], (6.2578747639, 1.8296715337), math.radians(45.0))
    assert sparse["request_poses"][1] == (current["request_poses"][1][1],)


def test_track3_sparse_never_dispatches_p1_m_or_dense_arc_samples():
    sparse = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "track3_sparse_exit"
    )
    dispatched = [item for request in sparse["request_poses"] for item in request]
    p1 = sparse["logical_points"]["P1"]
    m = sparse["fillet"]["arc_points"][len(sparse["fillet"]["arc_points"]) // 2]
    assert all((point[0][0], point[0][1]) != (p1["x_m"], p1["y_m"])
               for point in dispatched)
    assert all((point[0][0], point[0][1]) != pytest.approx(m)
               for point in dispatched)
    assert len(dispatched) == 3


def test_track3_matrix_contract_keeps_planner_radius_and_p1_measurement_metadata():
    geometry = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "track3_sparse_exit"
    )
    assert geometry["planner_minimum_turning_radius_m"] == 4.0
    assert geometry["track3_nominal_radius_m"] == 8.0
    assert geometry["logical_points"]["P1"]["x_m"] == pytest.approx(5.5)


def test_exact_productive_replay_preserves_captured_poses_and_yaws():
    geometry = _experiment_geometry(
        Pose2D(5.274779424261093, 0.7033543116419798, 0.24426658083476302),
        GOAL, "track3_exact_productive_replay"
    )
    assert tuple(len(request) for request in geometry["request_poses"]) == (6, 5)
    request_a, request_b = geometry["request_poses"]
    assert request_a[0][0] == pytest.approx((2.156815699476283, 0.20325882267206907))
    assert request_a[0][1] == pytest.approx(math.radians(0.13465437977510875))
    assert request_b[0][0] == pytest.approx((6.865496630217754, 1.9788015772680372))
    assert request_b[0][1] == pytest.approx(math.radians(38.34738883114576))
    assert request_b[-1][0] == pytest.approx((9.08356002620621, 4.204101139282841))
    assert request_b[-1][1] == pytest.approx(math.radians(45.09330802155037))
    assert geometry["planner_minimum_turning_radius_m"] == pytest.approx(4.0)
    assert geometry["replay_source"]["chunk_id"] == "1"


def test_request_b_delta_debug_changes_only_the_allowed_contract():
    pose = Pose2D(0.0, 0.0, 0.0)
    full = _experiment_geometry(pose, GOAL, "track3_request_b_full5")
    endpoints = _experiment_geometry(pose, GOAL, "track3_request_b_endpoints_only")
    normalized = _experiment_geometry(pose, GOAL, "track3_request_b_normalize_b0_yaw")
    dropped = _experiment_geometry(pose, GOAL, "track3_request_b_drop_b0")
    assert full["request_poses"][0] == endpoints["request_poses"][0]
    assert full["request_poses"][0] == normalized["request_poses"][0]
    assert full["request_poses"][0] == dropped["request_poses"][0]
    full_b = full["request_poses"][1]
    assert endpoints["request_poses"][1] == (full_b[0], full_b[4])
    assert normalized["request_poses"][1][0][0] == full_b[0][0]
    assert normalized["request_poses"][1][0][1] == full_b[1][1]
    assert normalized["request_poses"][1][1:] == full_b[1:]
    assert dropped["request_poses"][1] == full_b[1:]
    assert endpoints["delta_debug_contract"]["changed_fields"] == ["B1-B3 removed"]
    assert normalized["delta_debug_contract"]["changed_fields"] == ["B0.yaw := B1.yaw"]
    assert dropped["delta_debug_contract"]["changed_fields"] == ["B0 removed"]


@pytest.mark.parametrize("variant, added_index", [
    ("track3_request_b_add_b1", 1),
    ("track3_request_b_add_b2", 2),
    ("track3_request_b_add_b3", 3),
])
def test_request_b_single_additions_are_exact_subsets_of_full5(
        variant, added_index):
    base = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "track3_request_b_endpoints_only"
    )
    full = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "track3_request_b_full5"
    )
    candidate = _experiment_geometry(Pose2D(0.0, 0.0, 0.0), GOAL, variant)
    assert candidate["request_poses"][0] == base["request_poses"][0]
    assert candidate["request_poses"][0] == full["request_poses"][0]
    assert candidate["request_poses"][1] == (
        full["request_poses"][1][0],
        full["request_poses"][1][added_index],
        full["request_poses"][1][4],
    )
    contract = candidate["delta_debug_contract"]
    assert contract["full5_indices_retained"] == [0, added_index, 4]
    assert contract["full5_indices_removed"] == [
        index for index in (1, 2, 3) if index != added_index
    ]
    assert [item["relation"] for item in contract["base_structured_diff"]] == [
        "retained", "added", "retained"
    ]
    assert all(item["yaw_equal"] for item in contract["full5_structured_diff"])


def test_request_b_full5_drop_b3_is_exact_confirmation_delta():
    full = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL, "track3_request_b_full5"
    )
    dropped = _experiment_geometry(
        Pose2D(0.0, 0.0, 0.0), GOAL,
        "track3_request_b_full5_drop_b3",
    )
    assert dropped["request_poses"][0] == full["request_poses"][0]
    assert dropped["request_poses"][1] == (
        full["request_poses"][1][0],
        full["request_poses"][1][1],
        full["request_poses"][1][2],
        full["request_poses"][1][4],
    )
    contract = dropped["delta_debug_contract"]
    assert contract["full5_indices_retained"] == [0, 1, 2, 4]
    assert contract["full5_indices_removed"] == [3]
    assert contract["changed_fields"] == ["B3 removed from FULL5"]
    assert all(item["xy_equal"] and item["yaw_equal"]
               for item in contract["full5_structured_diff"])
