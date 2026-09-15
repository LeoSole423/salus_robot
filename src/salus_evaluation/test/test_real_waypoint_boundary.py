import math

from salus_evaluation.real_waypoint_boundary_runner import (
    ARMS,
    _load_fixture,
)


def test_p0_fixture_is_track3_no_synthetic_two_key_request():
    fixture, poses = _load_fixture()

    assert fixture["scenario"].startswith("TRACK3")
    assert fixture["synthetic_count"] == 0
    assert fixture["planner"]["plugin"] == "SmacPlannerHybrid"
    assert fixture["planner"]["motion_model"] == "DUBIN"
    assert fixture["planner"]["minimum_turning_radius_m"] == 4.0
    assert fixture["request"]["input_indices"] == [2, 3]
    assert fixture["request"]["keys"] == [True, True]
    assert len(poses) == 2


def test_immediate_arm_keeps_p2_xy_and_effective_yaw_exactly():
    fixture, poses = _load_fixture()
    p2 = fixture["request"]["poses_xy"][0]
    p2_yaw = math.radians(fixture["request"]["yaws_deg"][0])

    assert ARMS["current_two_key"] == "CURRENT_TWO_KEY"
    assert ARMS["immediate_key_only_same_yaw"] == "IMMEDIATE_KEY_ONLY_SAME_YAW"
    assert poses[0][0] == tuple(p2)
    assert poses[0][1] == p2_yaw
    assert fixture["request"]["action_type"] == "NavigateThroughPoses"


def test_fixture_preserves_the_productive_dispatch_state_and_path_evidence():
    fixture, _poses = _load_fixture()
    dispatch = fixture["robot_pose_at_dispatch"]
    metrics = fixture["path_metrics"]

    assert all(math.isfinite(float(dispatch[key])) for key in ("x_m", "y_m", "yaw_rad"))
    assert metrics["self_intersections"] == 1
    assert metrics["length_m"] > metrics["direct_distance_m"]
