from dataclasses import replace
from math import nan

import pytest

from salus_navigation.patrol_domain import (
    BatteryReturnDisposition, PatrolMachine, PatrolMissionSpec, PatrolPhase,
    PatrolRoute, route_roles_for_phase, select_return_exit, validate_mission,
)
from salus_navigation.route_model import RouteWaypoint
from salus_navigation.route_chunker import build_chunk, next_start
from salus_navigation.route_preparation import prepare


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
    transition = machine.latch_battery_return(at_home=False)
    assert transition.disposition is BatteryReturnDisposition.RETURN_REQUESTED
    assert machine.state.phase is PatrolPhase.EXIT_LOOP
    assert machine.state.low_battery_active
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


def test_latched_battery_at_home_never_dispatches_departure():
    machine = PatrolMachine(spec(), "mission")
    assert machine.start(at_home=True, battery_return_latched=True) is PatrolPhase.AT_HOME
    assert machine.state.low_battery_active
    assert not machine.state.active


def test_battery_during_departure_stops_at_home_but_defers_when_away():
    at_home = PatrolMachine(spec(), "at-home")
    at_home.start(at_home=True)
    stopped = at_home.latch_battery_return(at_home=True)
    assert stopped.disposition is BatteryReturnDisposition.AT_HOME
    assert stopped.cancel_active_route
    assert at_home.state.phase is PatrolPhase.AT_HOME

    away = PatrolMachine(spec(), "away")
    away.start(at_home=True)
    deferred = away.latch_battery_return(at_home=False)
    assert deferred.disposition is BatteryReturnDisposition.DEFERRED_UNTIL_LOOP
    assert away.goal_succeeded() is PatrolPhase.JOIN_LOOP
    assert away.goal_succeeded() is PatrolPhase.EXIT_LOOP


def test_battery_during_join_is_deferred_then_selects_exit_once():
    machine = PatrolMachine(spec(depart=False), "mission")
    machine.start(at_home=False)
    transition = machine.latch_battery_return(at_home=False)
    assert transition.disposition is BatteryReturnDisposition.DEFERRED_UNTIL_LOOP
    assert machine.goal_succeeded() is PatrolPhase.EXIT_LOOP
    selected = machine.state.return_exit
    duplicate = machine.latch_battery_return(at_home=False)
    assert duplicate.disposition is BatteryReturnDisposition.ALREADY_RETURNING
    assert machine.state.return_exit == selected


def test_pause_preserves_battery_latch_without_automatic_resume():
    machine = PatrolMachine(spec(depart=False), "mission")
    machine.start(at_home=False)
    machine.latch_battery_return(at_home=False)
    machine.pause("manual takeover")
    recovered = machine.latch_battery_return(at_home=False)
    assert recovered.disposition is BatteryReturnDisposition.HELD_INACTIVE
    assert machine.state.phase is PatrolPhase.PAUSED
    assert machine.state.low_battery_active


def test_patrol_loop_uses_normal_roles_for_legacy_pair_continuity():
    route = spec().loop

    assert route_roles_for_phase(PatrolPhase.JOIN_LOOP, route) == (
        "normal", "normal", "normal")
    assert route_roles_for_phase(PatrolPhase.PATROL, route) == (
        "normal", "normal", "normal")
    assert route_roles_for_phase(PatrolPhase.EXIT_LOOP, route) == (
        "normal", "normal", "normal")


def test_actions_and_explicit_yaws_remain_hard_patrol_boundaries():
    automatic = point(0, 0)
    explicit = RouteWaypoint(**{
        **point(1, 10).__dict__, "yaw_explicit": True,
    })
    route = PatrolRoute(
        (automatic, explicit, point(2, 20)),
        ('[{"type":"brake_hold","duration_s":1.0,"brake_pct":30}]', "", ""),
    )

    assert route_roles_for_phase(PatrolPhase.PATROL, route) == (
        "hard", "hard", "normal")


def test_departure_and_return_home_keep_strict_terminal_boundaries():
    connector = PatrolRoute(
        (point(0, 0), point(1, 10), point(2, 20)),
        ("", "", ""),
    )

    assert route_roles_for_phase(PatrolPhase.DEPART_HOME, connector) == (
        "normal", "normal", "hard")
    assert route_roles_for_phase(PatrolPhase.RETURN_HOME, connector) == (
        "normal", "normal", "hard")


def test_phase_roles_compose_with_legacy_pair_without_softening_home():
    loop = spec().loop
    loop_roles = route_roles_for_phase(PatrolPhase.PATROL, loop)
    prepared_loop = prepare(
        [replace(point, role=role) for point, role in zip(
            loop.waypoints, loop_roles)],
        loop=True,
        input_count=len(loop.waypoints),
        spacing_m=35.0,
        chunk_span_m=120.0,
        chunk_max_waypoints=5,
    )
    first = build_chunk(prepared_loop, 0, mode="legacy_pair")
    second = build_chunk(
        prepared_loop,
        next_start(prepared_loop, first),
        mode="legacy_pair",
    )

    assert [point.input_index for point in first.waypoints] == [0, 1]
    assert [point.input_index for point in second.waypoints] == [2, 0]

    returning = PatrolRoute(
        (point(3, 12), spec().home),
        ("", ""),
    )
    return_roles = route_roles_for_phase(PatrolPhase.RETURN_HOME, returning)
    prepared_return = prepare(
        [replace(point, role=role) for point, role in zip(
            returning.waypoints, return_roles)],
        loop=False,
        input_count=len(returning.waypoints),
        spacing_m=35.0,
        chunk_span_m=120.0,
        chunk_max_waypoints=5,
    )
    home_chunk = build_chunk(prepared_return, 0, mode="legacy_pair")

    assert [point.input_index for point in home_chunk.waypoints] == [3, 9]
    assert home_chunk.waypoints[-1].role == "hard"
