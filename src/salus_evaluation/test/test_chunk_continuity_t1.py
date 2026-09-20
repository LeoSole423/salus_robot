"""Pure regression tests for the complete route continuity observation."""

from salus_evaluation.chunk_continuity_runner import (
    ROUTE_CHUNK_MAX_WAYPOINTS,
    ROUTE_CHUNK_SPAN_M,
    ROUTE_SPACING_M,
    _chunk_windows,
    _trajectory_metrics,
)


def event(code, stamp, generation=None):
    details = {}
    if generation is not None:
        details["goal_generation"] = generation
    return {"code": code, "stamp_s": stamp, "details": details}


def dispatch(chunk_id, stamp):
    x_by_chunk = {"A": 0.0, "B": 1.0}
    return {
        "chunk_id": chunk_id,
        "stamp_s": stamp,
        "poses_xy": [[x_by_chunk[chunk_id], 0.0]],
    }


def plan(stamp, x):
    return {"stamp_s": stamp, "points": ((x, 0.0), (x + 1.0, 0.0))}


def odom(stamp, x):
    return {"stamp_s": stamp, "x_m": x, "y_m": 0.0, "yaw_rad": 0.0}


def test_all_plans_are_kept_and_associated_with_goal_generation_windows():
    windows = _chunk_windows(
        [dispatch("A", 1.0), dispatch("B", 3.0)],
        [
            event("GOAL_ACCEPTED", 1.1, 10),
            event("GOAL_RESULT_SUCCEEDED", 2.9, 10),
            event("GOAL_ACCEPTED", 3.1, 11),
            event("GOAL_RESULT_SUCCEEDED", 4.9, 11),
        ],
        [plan(1.2, 0.0), plan(1.8, 0.2), plan(3.2, 1.0), plan(4.2, 1.2)],
        [odom(1.2, 0.0), odom(1.8, 0.4), odom(3.2, 1.0), odom(4.2, 1.4)],
        ((0.0, 0.0), (1.0, 0.0)),
    )

    assert [window["goal_generation"] for window in windows] == [10, 11]
    assert [[item["stamp_s"] for item in window["plans"]] for window in windows] == [
        [1.2, 1.8],
        [3.2, 4.2],
    ]
    assert windows[0]["plans"][0]["odometry_global_near_plan"]["x_m"] == 0.0
    assert windows[1]["plans"][1]["odometry_global_near_plan"]["x_m"] == 1.4


def test_executed_trajectory_is_measured_separately_from_planner_path():
    samples = [odom(1.0, 0.0), odom(2.0, 1.0), odom(3.0, 2.0)]

    result = _trajectory_metrics(samples, (2.0, 0.0), 0.0)

    assert result["available"] is True
    assert result["sample_count"] == 3
    assert result["length_m"] == 2.0
    assert result["final_error_m"] == 0.0
    assert result["final_yaw_error_rad"] == 0.0


def test_route_runner_uses_production_chunk_defaults_for_both_modes():
    assert ROUTE_SPACING_M == 35.0
    assert ROUTE_CHUNK_SPAN_M == 120.0
    assert ROUTE_CHUNK_MAX_WAYPOINTS == 5
