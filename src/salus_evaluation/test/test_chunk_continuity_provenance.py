import math

import pytest

from salus_evaluation.chunk_continuity_runner import (
    ROUTE_CHUNK_MAX_WAYPOINTS,
    ROUTE_CHUNK_SPAN_M,
    TURN_RADIUS_M,
    _chunk_windows,
    _transition_metrics,
    _validated_route_spacing,
    _wide_turn_local_route,
    ChunkContinuityRunner,
)


def _event(stamp, code, generation=None):
    details = {}
    if generation is not None:
        details["goal_generation"] = str(generation)
    return {"stamp_s": stamp, "code": code, "details": details}


def _plan(stamp, points):
    return {"stamp_s": stamp, "points": points}


def test_chunk_windows_keep_all_plans_and_match_goal_generation():
    dispatches = [
        {"chunk_id": "A", "stamp_s": 1.0},
        {"chunk_id": "B", "stamp_s": 5.0},
    ]
    events = [
        _event(1.1, "GOAL_ACCEPTED", 7),
        _event(4.0, "GOAL_RESULT_SUCCEEDED", 7),
        _event(5.1, "GOAL_ACCEPTED", 8),
        _event(8.0, "GOAL_RESULT_ABORTED", 8),
    ]
    plans = [
        _plan(1.5, ((0.0, 0.0), (1.0, 0.0))),
        _plan(2.0, ((0.0, 0.0), (1.0, 0.1))),
        _plan(3.5, ((0.0, 0.0), (1.0, 0.2))),
        _plan(5.2, ((1.0, 0.0), (2.0, 0.0))),
        _plan(6.0, ((1.0, 0.0), (2.0, 0.1))),
    ]
    odometry = [
        {"stamp_s": 1.0, "x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
        {"stamp_s": 2.1, "x_m": 0.3, "y_m": 0.0, "yaw_rad": 0.1},
        {"stamp_s": 4.1, "x_m": 0.8, "y_m": 0.0, "yaw_rad": 0.2},
        {"stamp_s": 5.0, "x_m": 1.0, "y_m": 0.0, "yaw_rad": 0.3},
        {"stamp_s": 6.1, "x_m": 1.4, "y_m": 0.0, "yaw_rad": 0.4},
    ]

    windows = _chunk_windows(
        dispatches, events, plans, odometry, ((0.0, 0.0), (2.0, 0.0))
    )

    assert [len(window["plans"]) for window in windows] == [3, 2]
    assert windows[0]["goal_generation"] == 7
    assert windows[0]["goal_result"]["code"] == "GOAL_RESULT_SUCCEEDED"
    assert windows[1]["goal_generation"] == 8
    assert windows[0]["plans"][1]["odometry_global_near_plan"]["stamp_s"] == 2.1
    assert windows[0]["odometry_global_near_dispatch"]["stamp_s"] == 1.0
    assert windows[0]["odometry_global_near_result"]["stamp_s"] == 4.1


def test_transition_does_not_count_cross_plan_intersections_as_self_intersections():
    plan_a = ((0.0, 0.0), (2.0, 2.0))
    plan_b = ((0.0, 2.0), (2.0, 0.0))

    metrics = _transition_metrics(plan_a, plan_b, ((0.0, 0.0), (2.0, 0.0)))

    assert metrics["self_intersections_plan_A"] == 0
    assert metrics["self_intersections_plan_B"] == 0
    assert metrics["cross_intersections_A_B"] == 1
    assert metrics["self_intersections"] is None
    assert metrics["self_intersections_note"] == (
        "not computed across independent plans"
    )


def test_plan_metrics_include_geometry_quality_without_changing_topology_metric():
    from salus_evaluation.chunk_continuity_runner import _plan_metrics

    metrics = _plan_metrics(((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)),
                            ((0.0, 0.0), (2.0, 0.0)))
    assert metrics["self_intersections"] == 0
    assert metrics["geometry_quality"]["total_heading_variation_rad"] == 0.0


def test_track3_spacing_contract_has_dense_only_one_metre_synthetics():
    from salus_navigation.route_model import RouteWaypoint
    from salus_navigation.route_preparation import expand

    points = _wide_turn_local_route()
    assert TURN_RADIUS_M == 8.0
    assert math.dist(points[0], points[1]) == pytest.approx(4.1411047216)
    assert math.dist(points[1], points[2]) == pytest.approx(4.1411047216)
    assert math.dist(points[2], points[3]) == pytest.approx(4.1411047216)
    waypoints = [
        RouteWaypoint(0.0, 0.0, 0.0, index, map_x=x, map_y=y)
        for index, (x, y) in enumerate(points)
    ]
    synthetic_counts = {
        spacing: sum(
            not point.key
            for point in expand(waypoints, spacing, loop=False)
        )
        for spacing in (1.0, 5.0, 35.0)
    }

    assert synthetic_counts == {1.0: 12, 5.0: 0, 35.0: 0}


@pytest.mark.parametrize("value", (0.0, -1.0, float("nan"), float("inf")))
def test_route_spacing_rejects_non_finite_or_non_positive_values(value):
    with pytest.raises(ValueError, match="finite and positive"):
        _validated_route_spacing(value)


def test_route_runner_request_uses_selected_spacing_without_changing_fixed_fields():
    runner = object.__new__(ChunkContinuityRunner)
    runner.route_spacing_m = 5.0
    request = runner._request(((0.0, 0.0), (4.0, 0.0)))

    assert request.leg_spacing_m == 5.0
    assert request.chunk_span_m == ROUTE_CHUNK_SPAN_M
    assert request.chunk_max_waypoints == ROUTE_CHUNK_MAX_WAYPOINTS
    assert request.loop is False
    assert all(math.isnan(value) for value in request.yaws_deg)
