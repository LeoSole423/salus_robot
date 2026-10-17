"""Analytic geometry fixtures and monotonic deadline boundaries (no ROS)."""

import math
import pytest

from salus_navigation.maneuver_guard import (
    ManeuverConfig, ManeuverWait, inspect_maneuver,
)


def arc(degrees, radius=4.0, sign=1, step=2):
    return tuple((radius * math.sin(math.radians(t)),
                  sign * radius * (1 - math.cos(math.radians(t))))
                 for t in range(0, degrees + 1, step))


@pytest.mark.parametrize('degrees', [0, 30, 90, 120])
@pytest.mark.parametrize('sign', [-1, 1])
def test_straight_and_steep_valid_curves(degrees, sign):
    points = arc(degrees, sign=sign) if degrees else ((0, 0), (20, 0))
    assert not inspect_maneuver(points, robot_yaw=0).reason


@pytest.mark.parametrize('sign', [-1, 1])
def test_u_turn_requires_both_opposing_tangent_and_return(sign):
    result = inspect_maneuver(arc(180, sign=sign), robot_yaw=0)
    assert result.reason == 'PLANNER_U_TURN'
    assert result.return_m >= 1.17
    # A sharp heading alone without a body-length return is not sufficient.
    assert not inspect_maneuver(((0, 0), (10, 0), (9.9, sign * .1)), robot_yaw=0).reason


@pytest.mark.parametrize('step', [1, 2, 5])
@pytest.mark.parametrize('sign', [-1, 1])
def test_inserted_loop_rejected_at_multiple_sampling_densities(step, sign):
    points = (*arc(360, step=step, sign=sign), (20, 0))
    assert inspect_maneuver(points, robot_yaw=0, goals=((20, 0),)).reason == 'PLANNER_LOOP'


def test_intentional_closed_patrol_is_partitioned_by_ordered_poses():
    points = arc(360)
    goals = tuple(points[i] for i in (45, 90, 135, 180))
    assert not inspect_maneuver(points, robot_yaw=0, goals=goals).reason
    # An explicit circuit is no blanket exemption for an inserted second lap.
    bad = (*points, *points[1:])
    assert inspect_maneuver(bad, robot_yaw=0, goals=goals).reason == 'PLANNER_LOOP'


def test_s_curve_does_not_accumulate_unsigned_turn_as_a_loop():
    points = (*arc(90), *((4 + x, 4 - y) for x, y in arc(90)[1:]))
    assert not inspect_maneuver(points, robot_yaw=0).reason


def test_detection_is_rotation_and_translation_invariant():
    points = arc(180)
    for angle in (0, 1, 2, -2.5):
        rotated = tuple((100 + x * math.cos(angle) - y * math.sin(angle),
                         -50 + x * math.sin(angle) + y * math.cos(angle)) for x, y in points)
        assert inspect_maneuver(rotated, robot_yaw=angle).reason == 'PLANNER_U_TURN'


def test_robot_heading_detects_initial_reverse_request():
    assert inspect_maneuver(((0, 0), (-5, 0)), robot_yaw=0).reason == 'PLANNER_U_TURN'
    assert not inspect_maneuver(((0, 0), (-5, 0)), robot_yaw=math.pi).reason


def test_return_threshold_boundary():
    config = ManeuverConfig(return_m=1.17)
    assert not inspect_maneuver(((0, 0), (10, 0), (8.831, 0)), robot_yaw=0, config=config).reason
    assert inspect_maneuver(((0, 0), (10, 0), (8.829, 0)), robot_yaw=0, config=config).reason


def test_deadline_is_not_renewed_by_candidates_causes_or_stale_data():
    wait = ManeuverWait(ManeuverConfig(wait_s=10))
    assert wait.observe(now_s=2, reason='PLANNER_LOOP').remaining_s == 10
    assert wait.observe(now_s=4, reason='PLANNER_U_TURN', candidate=True).remaining_s == 8
    assert wait.observe(now_s=8, valid=False).remaining_s == 4
    final = wait.observe(now_s=12, key='good', candidate=True)
    assert final.state == 'NEEDS_OPERATOR' and final.reason == 'PLANNER_LOOP'
    assert wait.observe(now_s=100, key='good').state == 'NEEDS_OPERATOR'


def test_candidate_cannot_release_last_active_path_until_adoption():
    wait = ManeuverWait()
    wait.observe(now_s=0, reason='PLANNER_U_TURN', key='old')
    assert wait.observe(now_s=1, key='good', candidate=True).state == 'WAITING_REPLAN'
    assert wait.observe(now_s=2, key='old').state == 'WAITING_REPLAN'
    assert wait.observe(now_s=3, key='good').state == 'CLEAR'
    # A later, separate incident gets a fresh window after actual adoption.
    assert wait.observe(now_s=4, reason='PLANNER_LOOP').remaining_s == 15


def test_timeout_advances_without_bt_requests_and_reset_is_explicit():
    wait = ManeuverWait(ManeuverConfig(wait_s=2))
    wait.observe(now_s=10, reason='PLANNER_LOOP')
    assert wait.snapshot(12).state == 'NEEDS_OPERATOR'
    wait.reset()
    assert wait.snapshot(100).state == 'CLEAR'


@pytest.mark.parametrize('kwargs', [{'wait_s': 0}, {'return_m': -1}, {'loop_rad': math.pi},
                                    {'opposing_rad': float('nan')}, {'wait_s': float('inf')}])
def test_invalid_configuration_fails_closed(kwargs):
    with pytest.raises(ValueError):
        ManeuverConfig(**kwargs)


def test_cursor_cannot_jump_to_future_loop_crossing_nearer_to_robot():
    from salus_navigation.maneuver_guard import forward_suffix
    points = ((.1, 0), (10, 0), (10, 10), (0, 10), (0, 0), (20, 0))
    suffix, progress = forward_suffix(points, (0, 0))
    assert progress == 0
    assert len(suffix) == len(points)
    assert inspect_maneuver(suffix, robot_yaw=0).reason == 'PLANNER_LOOP'
    # Normal progress only consumes the bounded, already traversed prefix.
    suffix, progress = forward_suffix(((0, 0), (20, 0)), (8, 0), 6)
    assert progress == 8 and suffix[0] == (8, 0)


def test_loop_after_opposite_turn_is_not_hidden_by_signed_cancellation():
    prefix = arc(180)
    loop = tuple((-x, 8 - y) for x, y in arc(360, sign=-1)[1:])
    assert inspect_maneuver((*prefix, *loop), robot_yaw=0).reason == 'PLANNER_LOOP'


def test_open_three_quarter_arc_is_not_mislabeled_as_a_closed_loop():
    assert inspect_maneuver(arc(270), robot_yaw=0).reason != 'PLANNER_LOOP'
