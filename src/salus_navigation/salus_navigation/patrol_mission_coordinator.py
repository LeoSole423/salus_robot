"""ROS adapter for structured patrol, HOME and return requests.

The node converts and persists the complete patrol document once, then hands
each executable leg to ``route_executor``.  It never publishes velocity or
talks directly to Nav2.
"""
from __future__ import annotations

from dataclasses import replace
from math import hypot, isfinite, nan
from pathlib import Path
import threading
import uuid

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_localization.srv import FromLL
from salus_interfaces.msg import NavEvent
from salus_interfaces.srv import (
    CancelPatrolMission, CancelRouteMission, GetPatrolMissionState,
    GetRouteMissionState, RequestReturnHome, SetPatrolMissionLL,
    SetRouteMissionLL,
)

from .nav_command_server import diagnostic_level
from .patrol_domain import PatrolMachine, PatrolMissionSpec, PatrolPhase, PatrolRoute
from .patrol_store import write_atomic
from .route_actions import parse_actions
from .route_model import RouteWaypoint
from .route_preparation import resolve_yaws


def _route(lats, lons, yaws, actions, label, *, allow_empty):
    """Validate one public LL route without ROS I/O."""
    lat_values, lon_values = [float(v) for v in lats], [float(v) for v in lons]
    if not lat_values and not allow_empty:
        return None, f"{label} must contain at least one waypoint"
    if len(lat_values) != len(lon_values):
        return None, f"{label} lats and lons must have the same length"
    if any(not isfinite(v) for v in lat_values + lon_values):
        return None, f"{label} coordinates must be finite"
    yaw_values = [float(v) for v in yaws]
    if yaw_values and len(yaw_values) != len(lat_values):
        return None, f"{label} yaws_deg must be empty or match coordinates"
    if yaw_values and any(not isfinite(v) for v in yaw_values):
        return None, f"{label} yaws_deg must be finite when supplied"
    if not yaw_values:
        yaw_values = [nan] * len(lat_values)
    action_values = [str(v) for v in actions]
    if action_values and len(action_values) != len(lat_values):
        return None, f"{label} waypoint actions must match coordinates"
    if not action_values:
        action_values = [""] * len(lat_values)
    for index, action in enumerate(action_values):
        _, _, error = parse_actions(action, index)
        if error:
            return None, f"{label}: {error}"
    return PatrolRoute(
        tuple(RouteWaypoint(lat, lon, yaw, index, True, action)
              for index, (lat, lon, yaw, action) in
              enumerate(zip(lat_values, lon_values, yaw_values, action_values))),
        tuple(action_values),
    ), ""


def patrol_spec_from_request(request, defaults):
    """Build a pre-conversion mission, preserving the legacy public shape."""
    loop, error = _route(request.loop_lats, request.loop_lons, request.loop_yaws_deg,
                         request.loop_waypoint_action_jsons, "loop", allow_empty=False)
    if error:
        return None, error
    depart, error = _route(request.depart_lats, request.depart_lons, request.depart_yaws_deg,
                           request.depart_waypoint_action_jsons, "depart", allow_empty=True)
    if error:
        return None, error
    returning, error = _route(
        request.return_lats, request.return_lons, request.return_yaws_deg,
        request.return_waypoint_action_jsons, "return connector", allow_empty=True)
    if error:
        return None, error
    home = (
        float(
            request.home_lat), float(
            request.home_lon), float(
                request.home_yaw_deg))
    if not all(isfinite(value) for value in home):
        return None, "HOME waypoint must contain finite lat/lon/yaw"
    leg_default, span_default, max_default = defaults
    leg = float(request.leg_spacing_m) if float(
        request.leg_spacing_m) > 0.0 else leg_default
    span = float(request.chunk_span_m) if float(
        request.chunk_span_m) > 0.0 else span_default
    maximum = int(request.chunk_max_waypoints) or max_default
    try:
        spec = PatrolMissionSpec(
            home=RouteWaypoint(home[0], home[1], home[2], -1), loop=loop,
            depart=depart, returning=returning,
            depart_entry_loop_index=int(request.depart_entry_loop_index),
            leg_spacing_m=leg, chunk_span_m=span, chunk_max_waypoints=maximum,
        )
        PatrolMachine(spec, "validation")
    except ValueError as exc:
        return None, str(exc)
    return spec, ""


def _mapped(point, map_x, map_y):
    return replace(point, map_x=map_x, map_y=map_y)


def _normalise(route, loop):
    points, actions = list(route.waypoints), list(route.actions)
    if loop and len(points) > 2 and points[0].distance_to(points[-1]) <= 0.05:
        points.pop()
        actions.pop()
    return PatrolRoute(tuple(resolve_yaws(points, loop)), tuple(actions))


def resolved_patrol_spec(spec, converted):
    """Resolve yaws only after LL->map conversion, then preserve all actions."""
    return PatrolMissionSpec(
        home=converted["home"][0],
        loop=_normalise(
            PatrolRoute(
                tuple(
                    converted["loop"]),
                spec.loop.actions),
            True),
        depart=_normalise(
            PatrolRoute(
                tuple(
                    converted["depart"]),
                spec.depart.actions),
            False),
        returning=_normalise(
            PatrolRoute(
                tuple(
                    converted["return"]),
                spec.returning.actions),
            False),
        depart_entry_loop_index=spec.depart_entry_loop_index,
        leg_spacing_m=spec.leg_spacing_m,
        chunk_span_m=spec.chunk_span_m,
        chunk_max_waypoints=spec.chunk_max_waypoints,
        version=spec.version,
    )


class PatrolMissionCoordinator(Node):
    """ROS I/O for PatrolMachine; route_executor owns route execution."""

    def __init__(self):
        super().__init__("patrol_mission_coordinator")
        self.declare_parameter("fromll_timeout_s", 2.0)
        self.declare_parameter("state_poll_s", 0.2)
        self.declare_parameter("at_home_tolerance_m", 1.2)
        self.declare_parameter("default_leg_spacing_m", 2.0)
        self.declare_parameter("default_chunk_span_m", 20.0)
        self.declare_parameter("default_chunk_max_waypoints", 5)
        self.declare_parameter("runtime_dir", "runtime/patrol")
        self._lock = threading.RLock()
        self._machine = None
        self._spec = None
        self._preparation = None
        self._route_mission_id = ""
        self._delegated_input_indices = []
        self._route_state_future = None
        self._last_status, self._last_error = "IDLE", ""
        self._pose = None
        self._event_id = 0
        group = ReentrantCallbackGroup()
        self._fromll = [
            self.create_client(
                FromLL,
                "/fromLL",
                callback_group=group),
            self.create_client(
                FromLL,
                "/navsat_transform/fromLL",
                callback_group=group),
        ]
        self._set_route = self.create_client(
            SetRouteMissionLL,
            "/route_executor/set_route_mission_ll",
            callback_group=group)
        self._cancel_route = self.create_client(
            CancelRouteMission,
            "/route_executor/cancel_route_mission",
            callback_group=group)
        self._route_state = self.create_client(
            GetRouteMissionState,
            "/route_executor/get_route_mission_state",
            callback_group=group)
        self.create_subscription(
            Odometry, "/odometry/global", self._on_pose, 10)
        self.create_subscription(
            NavEvent,
            "/nav_command_server/events",
            self._on_event,
            10)
        self._events = self.create_publisher(
            NavEvent, "/nav_command_server/events", 10)
        self.create_service(
            SetPatrolMissionLL,
            "/route_executor/set_patrol_mission_ll",
            self._set)
        self.create_service(
            CancelPatrolMission,
            "/route_executor/cancel_patrol_mission",
            self._cancel)
        self.create_service(
            GetPatrolMissionState,
            "/route_executor/get_patrol_mission_state",
            self._state)
        self.create_service(
            RequestReturnHome,
            "/route_executor/request_return_home",
            self._request_return)
        self.create_timer(0.1, self._tick_preparation)
        self.create_timer(
            float(
                self.get_parameter("state_poll_s").value),
            self._tick_route_state)

    def _defaults(self):
        return (float(self.get_parameter("default_leg_spacing_m").value),
                float(self.get_parameter("default_chunk_span_m").value),
                int(self.get_parameter("default_chunk_max_waypoints").value))

    def _on_pose(self, message):
        self._pose = message.pose.pose.position

    def _set(self, request, response):
        spec, error = patrol_spec_from_request(request, self._defaults())
        response.loop_input_waypoint_count = 0 if spec is None else len(
            spec.loop.waypoints)
        response.loop_expanded_waypoint_count = 0
        if error:
            response.ok, response.error = False, error
            return response
        points = [("home", spec.home)]
        for label, route in (
                ("loop", spec.loop), ("depart", spec.depart), ("return", spec.returning)):
            points.extend((label, point) for point in route.waypoints)
        with self._lock:
            self._preparation = {
                "spec": spec,
                "points": points,
                "next": 0,
                "future": None,
                "deadline": self._now() +
                float(
                    self.get_parameter("fromll_timeout_s").value),
                "converted": {
                    "home": [],
                    "loop": [],
                    "depart": [],
                    "return": []}}
        response.ok, response.error = True, "preparing patrol conversion"
        return response

    def _tick_preparation(self):
        with self._lock:
            job = self._preparation
            if job is None:
                return
            future = job["future"]
            if future is not None and future.done():
                try:
                    result = future.result()
                    if result is None:
                        raise RuntimeError("empty fromLL response")
                    label, point = job["points"][job["next"]]
                    job["converted"][label].append(
                        _mapped(point, result.map_point.x, result.map_point.y))
                    job["next"] += 1
                    job["future"] = None
                    job["deadline"] = self._now(
                    ) + float(self.get_parameter("fromll_timeout_s").value)
                except Exception as exc:
                    self._reject(f"patrol conversion failed: {exc}")
                    return
            elif future is not None and self._now() > job["deadline"]:
                self._reject("patrol conversion timed out")
                return
            if job["future"] is not None:
                return
            if job["next"] == len(job["points"]):
                self._preparation = None
                self._activate_spec(
                    resolved_patrol_spec(
                        job["spec"], job["converted"]))
                return
            client = next(
                (item for item in self._fromll if item.service_is_ready()), None)
            if client is None:
                if self._now() > job["deadline"]:
                    self._reject("fromLL unavailable for patrol")
                return
            _, point = job["points"][job["next"]]
            conversion = FromLL.Request()
            conversion.ll_point.latitude = point.lat
            conversion.ll_point.longitude = point.lon
            conversion.ll_point.altitude = 0.0
            job["future"] = client.call_async(conversion)

    def _reject(self, error):
        self._preparation, self._last_error = None, error
        self._event(DiagnosticStatus.ERROR, "PATROL_MISSION_REJECTED", error)

    def _activate_spec(self, spec):
        if self._pose is None:
            self._reject("global pose unavailable; patrol was not started")
            return
        if self._machine is not None and self._machine.state.active:
            future = self._cancel_route.call_async(
                CancelRouteMission.Request())
            future.add_done_callback(
                lambda done: self._finish_replacement(
                    done, spec))
            return
        self._start_spec(spec)

    def _finish_replacement(self, future, spec):
        try:
            result = future.result()
            if result is None or not result.ok:
                raise RuntimeError(
                    "empty cancellation response" if result is None else result.error)
        except Exception as exc:
            self._reject(f"previous patrol could not be cancelled: {exc}")
            return
        self._start_spec(spec)

    def _start_spec(self, spec):
        mission = PatrolMachine(spec, str(uuid.uuid4()))
        at_home = hypot(
            self._pose.x -
            spec.home.map_x,
            self._pose.y -
            spec.home.map_y) <= float(
            self.get_parameter("at_home_tolerance_m").value)
        phase = mission.start(at_home=at_home)
        try:
            write_atomic(Path(str(self.get_parameter(
                "runtime_dir").value)) / "patrol_mission.json", spec)
        except OSError as exc:
            self._reject(f"could not persist patrol mission: {exc}")
            return
        self._machine, self._spec, self._route_mission_id = mission, spec, ""
        self._last_status, self._last_error = phase.value, ""
        self._event(
            DiagnosticStatus.OK,
            "PATROL_MISSION_STARTED",
            "structured patrol started",
            mission_id=mission.state.mission_id,
            phase=phase.value,
            at_home=at_home)
        self._dispatch_phase()

    def _dispatch_phase(self):
        machine = self._machine
        if machine is None or not machine.state.active:
            return
        route, loop, _ = machine.current_route()
        # SetRouteMissionLL has no original-index field.  Keep the mapping at
        # this boundary so a rotated patrol loop can still match the return
        # exit selected by PatrolMachine.
        self._delegated_input_indices = [
            point.input_index for point in route.waypoints]
        request = SetRouteMissionLL.Request()
        request.lats, request.lons = [
            p.lat for p in route.waypoints], [
            p.lon for p in route.waypoints]
        request.yaws_deg, request.waypoint_action_jsons = [
            p.yaw_deg for p in route.waypoints], list(route.actions)
        request.waypoint_roles, request.loop = [
            "normal"] * len(route.waypoints), loop
        request.leg_spacing_m = machine.spec.leg_spacing_m
        request.chunk_span_m = machine.spec.chunk_span_m
        request.chunk_max_waypoints = machine.spec.chunk_max_waypoints
        if not self._set_route.service_is_ready():
            machine.pause("route executor unavailable")
            self._last_error = machine.state.pause_reason
            return
        phase = machine.state.phase
        future = self._set_route.call_async(request)
        future.add_done_callback(
            lambda done: self._on_route_dispatched(
                done, phase))

    def _on_route_dispatched(self, future, phase):
        with self._lock:
            machine = self._machine
            if machine is None or machine.state.phase is not phase:
                return
            try:
                result = future.result()
                if result is None or not result.ok:
                    raise RuntimeError(
                        "empty route response" if result is None else result.error)
            except Exception as exc:
                machine.pause(f"route dispatch failed: {exc}")
                self._last_error = machine.state.pause_reason
                self._event(
                    DiagnosticStatus.ERROR,
                    "PATROL_PHASE_FAILED",
                    self._last_error,
                    phase=phase.value)
                return
            self._event(
                DiagnosticStatus.OK,
                "PATROL_PHASE_DISPATCHED",
                "patrol phase dispatched",
                phase=phase.value)

    def _tick_route_state(self):
        with self._lock:
            if (self._machine is None or not self._machine.state.active
                    or self._route_state_future is not None):
                return
            if not self._route_state.service_is_ready():
                return
            self._route_state_future = self._route_state.call_async(
                GetRouteMissionState.Request())
            self._route_state_future.add_done_callback(self._on_route_state)

    def _on_route_state(self, future):
        with self._lock:
            self._route_state_future = None
            machine = self._machine
            if machine is None or not machine.state.active:
                return
            try:
                state = future.result()
                if state is None or not state.ok:
                    return
            except Exception:
                return
            if not self._route_mission_id and state.mission_id:
                self._route_mission_id = state.mission_id
            if self._route_mission_id and state.mission_id != self._route_mission_id:
                return
            if state.status == "COMPLETED" and machine.state.phase is PatrolPhase.DEPART_HOME:
                machine.goal_succeeded()
                self._dispatch_phase()
            elif state.status == "COMPLETED" and machine.state.phase is PatrolPhase.RETURN_HOME:
                machine.goal_succeeded()
                self._last_status = machine.state.phase.value
                self._event(
                    DiagnosticStatus.OK,
                    "PATROL_AT_HOME",
                    "patrol arrived at HOME",
                    mission_id=machine.state.mission_id)
            elif state.status in ("PAUSED", "ABORTED", "CANCELLED"):
                machine.pause(f"route executor {state.status.lower()}")
                self._last_error = machine.state.pause_reason

    def _on_event(self, message):
        if message.component != "route_executor" or message.code != "ROUTE_CHECKPOINT_REACHED":
            return
        details = {entry.key: entry.value for entry in message.details}
        with self._lock:
            machine = self._machine
            if machine is None or machine.state.phase not in (
                    PatrolPhase.JOIN_LOOP, PatrolPhase.EXIT_LOOP):
                return
            # Do not attribute a checkpoint from an earlier/manual route to
            # this patrol while the delegated route has not identified itself.
            if not self._route_mission_id or details.get(
                    "mission_id") != self._route_mission_id:
                return
            delegated_index = int(details.get("input_index", "-1"))
            index = (
                self._delegated_input_indices[delegated_index] if 0 <= delegated_index < len(
                    self._delegated_input_indices) else -1)
            if machine.state.phase is PatrolPhase.JOIN_LOOP:
                machine.goal_succeeded(index)
                self._event(
                    DiagnosticStatus.OK,
                    "PATROL_LOOP_JOINED",
                    "patrol entered loop",
                    loop_index=index)
            elif (machine.state.return_exit is not None
                  and index == machine.state.return_exit.loop_index):
                machine.goal_succeeded(index)
                future = self._cancel_route.call_async(
                    CancelRouteMission.Request())
                future.add_done_callback(self._after_exit_cancelled)

    def _after_exit_cancelled(self, future):
        try:
            result = future.result()
            if result is None or not result.ok:
                raise RuntimeError(
                    "empty cancellation response" if result is None else result.error)
        except Exception as exc:
            with self._lock:
                if self._machine is not None:
                    self._machine.pause(
                        f"return exit cancellation failed: {exc}")
                    self._last_error = self._machine.state.pause_reason
            return
        with self._lock:
            if self._machine is not None and self._machine.state.phase is PatrolPhase.RETURN_HOME:
                self._route_mission_id = ""
                self._dispatch_phase()

    def _request_return(self, _request, response):
        with self._lock:
            machine = self._machine
            if machine is None:
                response.ok, response.error = False, "no active patrol mission"
            elif machine.state.phase in (PatrolPhase.EXIT_LOOP, PatrolPhase.RETURN_HOME):
                response.ok, response.error = True, ""
            elif machine.request_return_home("operator"):
                response.ok, response.error = True, ""
                self._event(
                    DiagnosticStatus.WARN,
                    "RETURN_HOME_REQUESTED",
                    "structured return HOME requested",
                    exit_index=machine.state.return_exit.loop_index)
            else:
                response.ok = False
                response.error = (
                    f"return HOME unavailable during {machine.state.phase.value}")
        return response

    def _cancel(self, _request, response):
        with self._lock:
            self._preparation = None
            machine = self._machine
            self._machine, self._route_mission_id = None, ""
            self._last_status, self._last_error = "IDLE", ""
            if machine is not None:
                self._cancel_route.call_async(CancelRouteMission.Request())
                self._event(
                    DiagnosticStatus.WARN,
                    "PATROL_MISSION_CANCELLED",
                    "patrol cancelled",
                    mission_id=machine.state.mission_id)
        response.ok, response.error = True, ""
        return response

    def _state(self, _request, response):
        with self._lock:
            machine, spec = self._machine, self._spec
            response.ok, response.error = True, self._last_error
            response.active = bool(machine and machine.state.active)
            response.phase = "idle" if machine is None else machine.state.public_phase
            response.low_battery_active = (
                False if machine is None else machine.state.low_battery_active)
            response.return_home_requested = (
                False if machine is None else machine.state.return_requested)
            response.return_home_active = (
                False if machine is None else machine.state.return_active)
            response.return_exit_loop_index = (
                -1 if machine is None or machine.state.return_exit is None
                else machine.state.return_exit.loop_index)
            response.depart_entry_loop_index = - \
                1 if spec is None else spec.depart_entry_loop_index
            response.home_available = spec is not None
            response.mission_id = "" if machine is None else machine.state.mission_id
            response.status = self._last_status if machine is None else machine.state.phase.value
            if spec is not None:
                response.home_lat = spec.home.lat
                response.home_lon = spec.home.lon
                response.home_yaw_deg = spec.home.yaw_deg
                response.loop_lats = [point.lat for point in spec.loop.waypoints]
                response.loop_lons = [point.lon for point in spec.loop.waypoints]
                response.loop_yaws_deg = [point.yaw_deg for point in spec.loop.waypoints]
                response.loop_action_jsons = list(spec.loop.actions)
                response.return_lats = [
                    point.lat for point in spec.returning.waypoints]
                response.return_lons = [
                    point.lon for point in spec.returning.waypoints]
                response.return_yaws_deg = [
                    point.yaw_deg for point in spec.returning.waypoints]
                response.return_action_jsons = list(spec.returning.actions)
                response.depart_lats = [point.lat for point in spec.depart.waypoints]
                response.depart_lons = [point.lon for point in spec.depart.waypoints]
                response.depart_yaws_deg = [
                    point.yaw_deg for point in spec.depart.waypoints]
                response.depart_action_jsons = list(spec.depart.actions)
            if machine is not None and machine.state.active:
                route, _, _ = machine.current_route()
                response.active_lats, response.active_lons = [
                    p.lat for p in route.waypoints], [
                    p.lon for p in route.waypoints]
                response.active_yaws_deg = [p.yaw_deg for p in route.waypoints]
        return response

    def _event(self, severity, code, message, **details):
        self._event_id += 1
        event = NavEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.severity, event.component = diagnostic_level(
            severity), "patrol_mission_coordinator"
        event.code, event.message, event.event_id = code, message, self._event_id
        event.details = [
            KeyValue(
                key=str(key),
                value=str(value)) for key,
            value in details.items()]
        self._events.publish(event)

    @staticmethod
    def _now():
        from time import monotonic
        return monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = PatrolMissionCoordinator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
