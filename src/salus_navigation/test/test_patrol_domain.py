from math import nan

import pytest

from salus_navigation.patrol_domain import (
    PatrolMachine, PatrolMissionSpec, PatrolPhase, PatrolRoute, select_return_exit,
    validate_mission,
)
from salus_navigation.route_model import RouteWaypoint


def point(index, x):
    return RouteWaypoint(-31.0 + index / 1000, -64.0, 0.0, index, map_x=x, map_y=0.0)


def spec(*, depart=True, returning=True):
    loop = (point(0, 0), point(1, 10), point(2, 20))
    return PatrolMissionSpec(
        home=point(9, -5), loop=PatrolRoute(loop, ("", "", "")),
        depart=PatrolRoute((point(3, -2),), ("",)) if depart else PatrolRoute((), ()),
        returning=PatrolRoute((point(4, 12),), ("",)) if returning else PatrolRoute((), ()),
        depart_entry_loop_index=0, leg_spacing_m=2.0, chunk_span_m=20.0,
        chunk_max_waypoints=5,
    )


def test_validation_is_atomic_and_rejects_invalid_actions():
    mission = spec()
    assert validate_mission(mission) == ""
    bad = PatrolMissionSpec(**{**mission.__dict__, "loop": PatrolRoute(
        mission.loop.waypoints, ('[{"type":"unsupported"}]', "", ""))})
    assert "unsupported" in validate_mission(bad)
    assert "HOME" in validate_mission(PatrolMissionSpec(**{**mission.__dict__, "home": RouteWaypoint(nan, 0, 0, 0)}))


def test_depart_join_patrol_and_latched_return_flow():
    machine = PatrolMachine(spec(), "mission")
    assert machine.start(at_home=True) is PatrolPhase.DEPART_HOME
    assert machine.goal_succeeded() is PatrolPhase.JOIN_LOOP
    assert machine.goal_succeeded() is PatrolPhase.PATROL
    assert machine.battery_guard(valid=True, recommended=True)
    assert machine.state.phase is PatrolPhase.EXIT_LOOP
    assert not machine.battery_guard(valid=True, recommended=False)
    exit_index = machine.state.return_exit.loop_index
    assert machine.goal_succeeded(exit_index) is PatrolPhase.RETURN_HOME
    assert machine.goal_succeeded() is PatrolPhase.AT_HOME


def test_return_exit_is_nearest_and_ties_keep_original_order():
    chosen = select_return_exit((point(0, 0), point(1, 10), point(2, 20)), point(7, 10))
    assert chosen.loop_index == 1
    tie = select_return_exit((point(0, 0), point(1, 10)), point(7, 5))
    assert tie.loop_index == 0


def test_start_away_from_home_joins_loop_and_pause_preserves_reason():
    machine = PatrolMachine(spec(), "mission")
    assert machine.start(at_home=False) is PatrolPhase.JOIN_LOOP
    machine.pause("manual takeover")
    assert machine.state.phase is PatrolPhase.PAUSED
    assert machine.state.pause_reason == "manual takeover"


def test_invalid_mission_cannot_construct_machine():
    invalid = PatrolMissionSpec(**{**spec().__dict__, "depart_entry_loop_index": 9})
    with pytest.raises(ValueError):
        PatrolMachine(invalid, "mission")
