"""Characterization of the U-shaped plans observed in the live Gazebo run."""

from salus_navigation.plan_regression import PlanRegressionTracker, detect_plan_regression


CHUNK = ((77.6425, 84.6796), (127.5143, 81.2171))


def test_observed_replan_returns_behind_robot():
    # 2026-09-24 /plan after far_obstacle_persistent, representative vertices.
    plan = ((87.125, 84.125), (90.34, 80.20), (87.125, 76.28),
            (73.96, 78.11), (72.97, 83.08), (77.63, 85.13),
            (93.18, 81.38), (127.375, 81.625))
    result = detect_plan_regression(plan, CHUNK, (87.3017, 84.0605))
    assert result is not None
    assert 14.0 < result.backward_m < 16.0
    assert result.detour_ratio > 1.5


def test_forward_detour_around_obstacle_is_not_flagged():
    plan = ((87.0, 84.0), (90.0, 78.0), (98.0, 76.0),
            (108.0, 81.0), (127.0, 81.0))
    assert detect_plan_regression(plan, CHUNK, (87.0, 84.0)) is None


def test_requested_hairpin_is_not_classified_as_planner_regression():
    chunk = ((0.0, 0.0), (20.0, 0.0), (20.0, 10.0), (0.0, 10.0))
    assert detect_plan_regression(chunk, chunk, (19.0, 0.0)) is None


def test_short_or_absent_route_context_is_inconclusive():
    assert detect_plan_regression(((0.0, 0.0),), CHUNK, (0.0, 0.0)) is None
    assert detect_plan_regression(((0.0, 0.0), (10.0, 0.0)), (), (0.0, 0.0)) is None


def test_repeated_plans_emit_only_state_transitions():
    tracker = PlanRegressionTracker()
    regression = detect_plan_regression(
        ((87.125, 84.125), (73.96, 78.11), (72.97, 83.08), (127.375, 81.625)),
        CHUNK, (87.3017, 84.0605))
    assert regression is not None
    assert tracker.observe(regression) == "detected"
    assert tracker.observe(regression) is None
    assert tracker.observe(None) == "cleared"
    assert tracker.observe(None) is None
    assert tracker.observe(regression) == "detected"
    tracker.reset()
    assert tracker.observe(regression) == "detected"
