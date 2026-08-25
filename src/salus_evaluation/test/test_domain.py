import math
from pathlib import Path

import pytest

from salus_evaluation.gates import GateState, functional_gates, performance_gate
from salus_evaluation.metrics import (absolute_goal, arrival_metrics,
                                      command_response_sign,
                                      localization_metrics, tracking_metrics)
from salus_evaluation.models import (GoalSpec, ExpectedTurn, Pose2D,
                                     TimedCommand, TimedPose)
from salus_evaluation.schema import load_scenario


def timed(stamp, x, y, yaw=0.0, yaw_rate=0.0):
    return TimedPose(stamp, Pose2D(x, y, yaw), angular_z_rps=yaw_rate)


def test_relative_goal_respects_spawn_heading():
    goal = GoalSpec("left", 2.0, 1.0, math.pi / 2, 10.0, ExpectedTurn.LEFT)
    result = absolute_goal(Pose2D(10.0, 20.0, math.pi / 2), goal)
    assert result.x_m == pytest.approx(9.0)
    assert result.y_m == pytest.approx(22.0)


def test_tracking_metrics_have_known_cross_track_error():
    result = tracking_metrics((timed(0, 0, 1), timed(1, 1, 1)),
                              (Pose2D(0, 0, 0), Pose2D(2, 0, 0)))
    assert result.cross_track_rms_m == pytest.approx(1.0)
    assert result.cross_track_p95_m == pytest.approx(1.0)


def test_command_response_detects_historical_opposite_turn_bug():
    commands = (TimedCommand(1.0, .5, -.3), TimedCommand(2.0, .5, -.2))
    result = command_response_sign(commands,
                                   (timed(1, 0, 0, yaw_rate=.2),
                                    timed(2, 0, 0, yaw_rate=.1)))
    assert result.eligible_count == 2
    assert result.mismatch_fraction == 1.0
    assert result.first_command_sign == -1
    assert result.first_response_sign == 1


def test_arrival_records_exit_and_post_success_travel():
    poses = (timed(0, 0, 0), timed(1, .95, 0), timed(2, 1.3, 0),
             timed(3, 1.02, 0), timed(4, 1.1, 0))
    result = arrival_metrics(poses, Pose2D(1, 0, 0), .1, success_s=3)
    assert result.first_entry_s == 1
    assert result.exits_after_entry == 1
    assert result.post_success_distance_m == pytest.approx(.08)
    assert result.overshoot_m == pytest.approx(.3)


def test_localization_uses_ground_truth_not_self_consistency():
    result = localization_metrics((timed(0, 0, 0), timed(1, 1, 0)),
                                  (timed(0, .2, 0), timed(1, 1.2, 0)))
    assert result.position_rmse_m == pytest.approx(.2)
    assert result.final_position_error_m == pytest.approx(.2)


def test_functional_sign_gate_fails_and_performance_starts_calibrating():
    signs = command_response_sign((TimedCommand(0, .5, .2),),
                                  (timed(0, 0, 0, yaw_rate=-.2),))
    gates = functional_gates(finite_data=True, plan_present=True,
                             terminal_success=True, final_distance_m=.1,
                             tolerance_m=.2, sign_metrics=signs,
                             reverse_observed=False, reverse_allowed=False,
                             expected_turn=ExpectedTurn.RIGHT)
    assert next(g for g in gates if g.name == "turn_sign").state == GateState.FAIL
    assert performance_gate("cte", .2).state == GateState.CALIBRATING
    assert performance_gate("cte", 1.21, baseline_p95=1.0).state == GateState.FAIL


def test_straight_gate_does_not_require_an_artificial_turn():
    signs = command_response_sign((), ())
    gates = functional_gates(finite_data=True, plan_present=True,
                             terminal_success=True, final_distance_m=.1,
                             tolerance_m=.2, sign_metrics=signs,
                             reverse_observed=False, reverse_allowed=False,
                             expected_turn=ExpectedTurn.STRAIGHT)
    assert next(g for g in gates if g.name == "turn_sign").state == GateState.PASS


def test_turn_gate_rejects_coherent_motion_in_the_wrong_requested_direction():
    signs = command_response_sign((TimedCommand(0, .5, .2),),
                                  (timed(.2, 0, 0, yaw_rate=.2),))
    gates = functional_gates(finite_data=True, plan_present=True,
                             terminal_success=True, final_distance_m=.1,
                             tolerance_m=.2, sign_metrics=signs,
                             reverse_observed=False, reverse_allowed=False,
                             expected_turn=ExpectedTurn.RIGHT)
    assert next(g for g in gates if g.name == "turn_sign").state == GateState.FAIL


def test_turn_gate_uses_initial_physical_response_not_later_corrections():
    commands = (TimedCommand(0, .5, -.2), TimedCommand(1, .5, .2))
    poses = (timed(.2, 0, 0, yaw_rate=-.2),
             timed(1.2, 0, 0, yaw_rate=-.2))
    signs = command_response_sign(commands, poses)
    gates = functional_gates(finite_data=True, plan_present=True,
                             terminal_success=True, final_distance_m=.1,
                             tolerance_m=.2, sign_metrics=signs,
                             reverse_observed=False, reverse_allowed=False,
                             expected_turn=ExpectedTurn.RIGHT)
    assert signs.mismatch_count == 1
    assert next(g for g in gates if g.name == "turn_sign").state == GateState.PASS


def test_all_shipped_scenarios_are_strictly_valid():
    root = Path(__file__).parents[1] / "config" / "scenarios"
    scenarios = [load_scenario(path) for path in root.glob("*.yaml")]
    assert {item.scenario_id for item in scenarios} == {
        "straight_5m", "right_quarter", "left_quarter", "arrival_short"}


def test_schema_rejects_unknown_fields(tmp_path):
    path = tmp_path / "invalid.yaml"
    path.write_text(
        "schema_version: 1\nid: x\nworld: free\n"
        "spawn: {x_m: 0, y_m: 0, yaw_rad: 0}\n"
        "goals: []\nsurprise: true\n"
    )
    with pytest.raises(ValueError, match="unknown"):
        load_scenario(path)
