"""ROS-free structured patrol, HOME and battery-return policies.

The runtime adapter supplies converted map coordinates and performs ROS I/O;
this module owns only immutable mission data and deterministic transitions.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import hypot, isfinite
from typing import Iterable

from .route_actions import parse_actions
from .route_model import RouteWaypoint


class PatrolPhase(str, Enum):
    IDLE = "IDLE"
    DEPART_HOME = "DEPART_HOME"
    JOIN_LOOP = "JOIN_LOOP"
    PATROL = "PATROL"
    EXIT_LOOP = "EXIT_LOOP"
    RETURN_HOME = "RETURN_HOME"
    AT_HOME = "AT_HOME"
    PAUSED = "PAUSED"
    ABORTED = "ABORTED"


PUBLIC_PHASE = {
    PatrolPhase.IDLE: "idle", PatrolPhase.DEPART_HOME: "depart_home",
    PatrolPhase.JOIN_LOOP: "loop_main", PatrolPhase.PATROL: "loop_main",
    PatrolPhase.EXIT_LOOP: "return_pending", PatrolPhase.RETURN_HOME: "return_connector",
    PatrolPhase.AT_HOME: "parked_home", PatrolPhase.PAUSED: "paused",
    PatrolPhase.ABORTED: "aborted",
}


@dataclass(frozen=True)
class PatrolRoute:
    waypoints: tuple[RouteWaypoint, ...]
    actions: tuple[str, ...]


@dataclass(frozen=True)
class PatrolMissionSpec:
    home: RouteWaypoint
    loop: PatrolRoute
    depart: PatrolRoute
    returning: PatrolRoute
    depart_entry_loop_index: int
    leg_spacing_m: float
    chunk_span_m: float
    chunk_max_waypoints: int
    version: int = 1


@dataclass(frozen=True)
class ReturnExit:
    loop_index: int
    waypoint: RouteWaypoint
    distance_m: float


@dataclass
class PatrolState:
    phase: PatrolPhase = PatrolPhase.IDLE
    mission_id: str = ""
    return_exit: ReturnExit | None = None
    return_reason: str = ""
    low_battery_active: bool = False
    pause_reason: str = ""

    @property
    def active(self) -> bool:
        return self.phase in {
            PatrolPhase.DEPART_HOME, PatrolPhase.JOIN_LOOP, PatrolPhase.PATROL,
            PatrolPhase.EXIT_LOOP, PatrolPhase.RETURN_HOME,
        }

    @property
    def public_phase(self) -> str:
        return PUBLIC_PHASE[self.phase]

    @property
    def return_requested(self) -> bool:
        return self.phase in {PatrolPhase.EXIT_LOOP, PatrolPhase.RETURN_HOME}

    @property
    def return_active(self) -> bool:
        return self.phase is PatrolPhase.RETURN_HOME


def _finite_waypoint(point: RouteWaypoint) -> bool:
    return isfinite(point.lat) and isfinite(point.lon)


def _validate_route(route: PatrolRoute, label: str, *, required: bool) -> str:
    if required and not route.waypoints:
        return f"{label} route must not be empty"
    if len(route.waypoints) != len(route.actions):
        return f"{label} actions length must match waypoints"
    for index, point in enumerate(route.waypoints):
        if not _finite_waypoint(point):
            return f"{label} waypoint {index} coordinates must be finite"
        error = parse_actions(route.actions[index], index)[2]
        if error:
            return f"{label}: {error}"
    return ""


def validate_mission(spec: PatrolMissionSpec) -> str:
    if spec.version != 1:
        return "unsupported patrol mission version"
    if not _finite_waypoint(spec.home):
        return "HOME coordinates must be finite"
    for route, label, required in (
        (spec.loop, "loop", True), (spec.depart, "depart", False),
        (spec.returning, "return", False),
    ):
        error = _validate_route(route, label, required=required)
        if error:
            return error
    if len(spec.loop.waypoints) < 2:
        return "loop requires at least two waypoints"
    if not 0 <= spec.depart_entry_loop_index < len(spec.loop.waypoints):
        return "depart_entry_loop_index is outside loop"
    if not all(isfinite(value) and value > 0.0 for value in
               (spec.leg_spacing_m, spec.chunk_span_m)):
        return "leg_spacing_m and chunk_span_m must be finite and positive"
    if spec.chunk_max_waypoints < 1:
        return "chunk_max_waypoints must be positive"
    return ""


def _distance(first: RouteWaypoint, second: RouteWaypoint) -> float:
    if None not in (first.map_x, first.map_y, second.map_x, second.map_y):
        return hypot(first.map_x - second.map_x, first.map_y - second.map_y)
    # Mission preparation normally supplies map coordinates. This fallback is
    # deterministic for unit tests and pre-conversion validation only.
    return hypot((first.lat - second.lat) * 111_320.0,
                 (first.lon - second.lon) * 111_320.0)


def select_return_exit(loop: Iterable[RouteWaypoint], reference: RouteWaypoint) -> ReturnExit | None:
    candidates = list(loop)
    if not candidates:
        return None
    index, waypoint = min(
        enumerate(candidates), key=lambda item: (_distance(item[1], reference), item[0]))
    return ReturnExit(index, waypoint, _distance(waypoint, reference))


class PatrolMachine:
    """Small deterministic state machine; no ROS clients, timers or files."""

    def __init__(self, spec: PatrolMissionSpec, mission_id: str) -> None:
        error = validate_mission(spec)
        if error:
            raise ValueError(error)
        self.spec = spec
        self.state = PatrolState(mission_id=mission_id)

    def start(self, *, at_home: bool) -> PatrolPhase:
        if self.state.phase is not PatrolPhase.IDLE:
            raise ValueError("patrol mission already started")
        self.state.phase = (PatrolPhase.DEPART_HOME
                            if at_home and self.spec.depart.waypoints
                            else PatrolPhase.JOIN_LOOP)
        return self.state.phase

    def current_route(self) -> tuple[PatrolRoute, bool, int]:
        phase = self.state.phase
        if phase is PatrolPhase.DEPART_HOME:
            destination = self.spec.loop.waypoints[self.spec.depart_entry_loop_index]
            return PatrolRoute(self.spec.depart.waypoints + (destination,),
                               self.spec.depart.actions + ("",)), False, 0
        if phase in (PatrolPhase.JOIN_LOOP, PatrolPhase.PATROL, PatrolPhase.EXIT_LOOP):
            start = ((self.spec.depart_entry_loop_index + 1) % len(self.spec.loop.waypoints)
                     if phase is PatrolPhase.JOIN_LOOP else 0)
            points = self.spec.loop.waypoints[start:] + self.spec.loop.waypoints[:start]
            actions = self.spec.loop.actions[start:] + self.spec.loop.actions[:start]
            return PatrolRoute(points, actions), True, start
        if phase is PatrolPhase.RETURN_HOME:
            return PatrolRoute(self.spec.returning.waypoints + (self.spec.home,),
                               self.spec.returning.actions + ("",)), False, 0
        raise ValueError(f"phase {phase.value} has no executable route")

    def request_return_home(self, reason: str) -> bool:
        if self.state.phase is not PatrolPhase.PATROL:
            return False
        reference = self.spec.returning.waypoints[0] if self.spec.returning.waypoints else self.spec.home
        self.state.return_exit = select_return_exit(self.spec.loop.waypoints, reference)
        self.state.return_reason = reason
        self.state.phase = PatrolPhase.EXIT_LOOP
        return True

    def battery_guard(self, *, valid: bool, recommended: bool) -> bool:
        if not valid or not recommended:
            return False
        self.state.low_battery_active = True
        return self.request_return_home("battery_guard")

    def goal_succeeded(self, reached_loop_input_index: int | None = None) -> PatrolPhase:
        phase = self.state.phase
        if phase is PatrolPhase.DEPART_HOME:
            self.state.phase = PatrolPhase.JOIN_LOOP
        elif phase is PatrolPhase.JOIN_LOOP:
            self.state.phase = PatrolPhase.PATROL
        elif phase is PatrolPhase.EXIT_LOOP:
            if reached_loop_input_index != self.state.return_exit.loop_index:
                return phase
            self.state.phase = PatrolPhase.RETURN_HOME
        elif phase is PatrolPhase.RETURN_HOME:
            self.state.phase = PatrolPhase.AT_HOME
        return self.state.phase

    def pause(self, reason: str) -> None:
        if self.state.active:
            self.state.phase, self.state.pause_reason = PatrolPhase.PAUSED, reason

    def abort(self, reason: str) -> None:
        self.state.phase, self.state.pause_reason = PatrolPhase.ABORTED, reason
