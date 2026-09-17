#!/usr/bin/env python3
"""Exercise the latched battery return through public patrol contracts."""
import math
import json
import os
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from salus_navigation.route_geometry import path_geometry_metrics
from salus_interfaces.msg import (
    BatteryMissionGuard, CmdVelFinal, NavEvent, NavTelemetry, PathHealth,
)
from salus_interfaces.srv import (
    CancelPatrolMission, GetPatrolMissionState, GetRouteMissionState,
    SetPatrolMissionLL,
)
from std_msgs.msg import String
from smoke_runtime import (
    AsyncServicePoller, SmokeRuntime, finite_odometry, has_increasing_stamps,
    subscribe_navigation_startup,
)


LAT, LON = -31.4858037, -64.2410570
PLAN_MIN_DIRECT_DISTANCE_M = 2.0
PLAN_MAX_DETOUR_RATIO = 3.0
UNEXECUTED_MAX_GOAL_LATENCY_S = 1.0
UNEXECUTED_MAX_DISPLACEMENT_M = 0.05
POSITIVE_COMMAND_EPSILON_MPS = 1.0e-3


def _path_record(message):
    return {
        "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
        + int(message.header.stamp.nanosec),
        "frame_id": str(message.header.frame_id),
        "points": [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
        ],
    }


def _event_details(details):
    decoded = dict(details)
    for key in ("input_indices", "synthetic_offsets", "yaws_deg", "poses_xy",
                "robot_xy", "goal_generation"):
        value = decoded.get(key)
        if not isinstance(value, str):
            continue
        try:
            decoded[key] = json.loads(value)
        except (TypeError, ValueError):
            pass
    return decoded


def _stamp_seconds(message):
    return float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1.0e-9


def _odom_record(message):
    pose = message.pose.pose
    return {
        "stamp_s": _stamp_seconds(message),
        "x": float(pose.position.x),
        "y": float(pose.position.y),
    }


def _polyline_distance(samples):
    return sum(
        math.hypot(current["x"] - previous["x"], current["y"] - previous["y"])
        for previous, current in zip(samples, samples[1:])
    )


def classify_unexecuted_plan(
    plan,
    goal,
    odometry,
    controller_status,
    *,
    xy_goal_tolerance=1.2,
    max_goal_latency_s=UNEXECUTED_MAX_GOAL_LATENCY_S,
    max_displacement_m=UNEXECUTED_MAX_DISPLACEMENT_M,
):
    """Classify a pathological plan only with complete same-goal evidence.

    Controller status is intentionally required in addition to final-command
    samples: a zero ``/cmd_vel_final`` alone cannot prove that the plan was not
    executed.  ``received_monotonic_s`` is used for controller samples because
    that topic has no ROS timestamp; goal/odometry association remains on ROS
    time.
    """
    checks = {
        "goal_associated": bool(goal),
        "initial_pose_available": False,
        "initial_within_xy_tolerance": False,
        "result_succeeded": False,
        "succeeded_immediate": False,
        "controller_status_available": False,
        "positive_controller_order": None,
        "odometry_available": False,
        "displacement_m": None,
        "no_significant_displacement": False,
    }
    if not goal:
        return {
            "classification": "EVIDENCE_INCOMPLETE",
            "evidence_complete": False,
            "checks": checks,
        }

    target = goal.get("target_xy")
    initial = goal.get("initial_pose")
    accepted_s = goal.get("accepted_stamp_s")
    result_s = goal.get("result_stamp_s")
    if target and initial:
        checks["initial_pose_available"] = True
        initial_distance = math.hypot(
            float(initial["x"]) - float(target[0]),
            float(initial["y"]) - float(target[1]),
        )
        checks["initial_distance_m"] = initial_distance
        checks["initial_within_xy_tolerance"] = initial_distance <= float(xy_goal_tolerance)
    result_text = str(goal.get("result_text", "")).lower()
    checks["result_succeeded"] = result_text == "succeeded"
    if accepted_s is not None and result_s is not None:
        latency = float(result_s) - float(accepted_s)
        checks["goal_latency_s"] = latency
        checks["succeeded_immediate"] = (
            checks["result_succeeded"]
            and 0.0 <= latency <= float(max_goal_latency_s)
        )

    if accepted_s is not None and result_s is not None:
        samples = [
            sample for sample in odometry
            if float(accepted_s) <= float(sample["stamp_s"]) <= float(result_s)
        ]
        if len(samples) >= 2:
            checks["odometry_available"] = True
            checks["displacement_m"] = _polyline_distance(samples)
            checks["no_significant_displacement"] = (
                checks["displacement_m"] <= float(max_displacement_m)
            )

    accepted_received = goal.get("accepted_received_monotonic_s")
    result_received = goal.get("result_received_monotonic_s")
    if accepted_received is not None and result_received is not None:
        statuses = [
            sample for sample in controller_status
            if accepted_received <= float(sample.get("received_monotonic_s", -1.0))
            <= result_received
        ]
        valid_statuses = [sample for sample in statuses if sample.get("valid", True)]
        checks["controller_status_available"] = bool(valid_statuses)
        if valid_statuses:
            positive = any(
                float(sample.get("requested_linear_x_mps", 0.0))
                > POSITIVE_COMMAND_EPSILON_MPS
                for sample in valid_statuses
            )
            checks["positive_controller_order"] = positive

    complete = all((checks["goal_associated"], checks["initial_pose_available"],
                    checks["initial_within_xy_tolerance"], checks["result_succeeded"],
                    checks["succeeded_immediate"], checks["controller_status_available"],
                    checks["positive_controller_order"] is False,
                    checks["odometry_available"], checks["no_significant_displacement"]))
    return {
        "classification": (
            "UNEXECUTED_WITHIN_TOLERANCE" if complete else "EVIDENCE_INCOMPLETE"
        ),
        "evidence_complete": complete,
        "checks": checks,
    }


def _goal_generation(event):
    value = event.get("details", {}).get("goal_generation")
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _goal_windows(events, odometry):
    """Associate dispatches, accepted/results and target pose without UUIDs."""
    dispatches = [event for event in events if event["code"] == "ROUTE_CHUNK_DISPATCHED"]
    windows = []
    for index, dispatch in enumerate(dispatches):
        next_dispatch_s = (
            dispatches[index + 1]["timestamp_s"]
            if index + 1 < len(dispatches) else float("inf")
        )
        accepted = next((event for event in events
                         if event["code"] == "GOAL_ACCEPTED"
                         and dispatch["timestamp_s"] <= event["timestamp_s"] < next_dispatch_s), None)
        if accepted is None:
            continue
        generation = _goal_generation(accepted)
        result = next((event for event in events
                       if event["code"] in {"GOAL_RESULT_SUCCEEDED", "GOAL_RESULT_ABORTED", "GOAL_CANCELLED"}
                       and accepted["timestamp_s"] <= event["timestamp_s"] < next_dispatch_s
                       and (generation is None or _goal_generation(event) == generation)), None)
        target = dispatch.get("details", {}).get("poses_xy")
        target = target[-1] if target else None
        initial = None
        if accepted is not None and odometry:
            initial = min(
                odometry,
                key=lambda sample: abs(sample["stamp_s"] - accepted["timestamp_s"]),
            )
        windows.append({
            "chunk_id": dispatch.get("details", {}).get("chunk_id"),
            "goal_generation": generation,
            "accepted_stamp_s": accepted["timestamp_s"],
            "accepted_received_monotonic_s": accepted.get("received_monotonic_s"),
            "result_stamp_s": None if result is None else result["timestamp_s"],
            "result_received_monotonic_s": None if result is None else result.get("received_monotonic_s"),
            "result_text": "" if result is None else result["details"].get("status", "succeeded" if result["code"] == "GOAL_RESULT_SUCCEEDED" else result["code"]),
            "target_xy": target,
            "initial_pose": initial,
        })
    return windows


def plan_topology_evidence(
    plans,
    chunks,
    *,
    events=None,
    odometry=None,
    controller_status=None,
    xy_goal_tolerance=1.2,
):
    """Measure every planner path against its active route chunk.

    The patrol itself is deliberately a loop.  This only rejects a loop *inside
    a single Nav2 plan*, where an Ackermann route in the obstacle-free smoke
    should stay topologically simple and reasonably direct.
    """
    observations = []
    goal_windows = _goal_windows(events or [], odometry or [])
    for plan in plans:
        candidates = [chunk for chunk in chunks if chunk["stamp_ns"] <= plan["stamp_ns"]]
        if not candidates or len(plan["points"]) < 2:
            continue
        chunk = candidates[-1]
        if len(chunk["points"]) < 1:
            continue
        reference = [plan["points"][0], *chunk["points"]]
        metrics = path_geometry_metrics(plan["points"], reference)
        excessive_detour = (
            metrics.direct_distance_m >= PLAN_MIN_DIRECT_DISTANCE_M
            and metrics.detour_ratio > PLAN_MAX_DETOUR_RATIO
        )
        goal = next(
            (
                window for window in goal_windows
                if window["accepted_stamp_s"] <= plan["stamp_ns"] / 1.0e9
                and window["result_stamp_s"] is not None
                and plan["stamp_ns"] / 1.0e9 <= window["result_stamp_s"]
            ),
            None,
        )
        execution = classify_unexecuted_plan(
            plan,
            goal,
            odometry or [],
            controller_status or [],
            xy_goal_tolerance=xy_goal_tolerance,
        )
        pathological = metrics.self_intersections > 0 or excessive_detour
        observations.append({
            "plan_frame": plan["frame_id"],
            "plan_poses": len(plan["points"]),
            "chunk_frame": chunk["frame_id"],
            "chunk_poses": len(chunk["points"]),
            "length_m": metrics.length_m,
            "direct_distance_m": metrics.direct_distance_m,
            "detour_ratio": metrics.detour_ratio,
            "max_deviation_m": metrics.max_deviation_m,
            "self_intersections": metrics.self_intersections,
            "excessive_detour": excessive_detour,
            "topology_ok": not pathological,
            "goal_generation": None if goal is None else goal["goal_generation"],
            "goal_chunk_id": None if goal is None else goal["chunk_id"],
            "execution": execution,
            "diagnostic_anomaly": (
                pathological
                and execution["classification"] == "UNEXECUTED_WITHIN_TOLERANCE"
            ),
            "gate_ok": (
                not pathological
                or execution["classification"] == "UNEXECUTED_WITHIN_TOLERANCE"
            ),
        })
    return observations


def assert_plan_topology(plans, chunks, **evidence):
    observations = plan_topology_evidence(plans, chunks, **evidence)
    if not observations:
        raise RuntimeError("no planner paths observed while active patrol was running")
    failures = [item for item in observations if not item["gate_ok"]]
    if failures:
        raise RuntimeError(f"unnecessary planner loop detected: {failures}")
    return observations


class Smoke(Node):
    def __init__(self):
        super().__init__(
            "patrol_battery_smoke",
            parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom = []
        self.events = []
        self.telemetry = []
        self.path_health = []
        self.final_commands = []
        self.controller_status = []
        self.plans = []
        self.active_chunks = []
        self.event_receipts = []
        self.capture_plan_topology = False
        self.guard_low = False
        self.guard_publications = 0
        self.last_guard_publish = 0.0
        self.phase_history = []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        self.create_subscription(
            NavEvent, "/nav_command_server/events", self._on_event, 10)
        self.create_subscription(
            NavTelemetry, "/nav_command_server/telemetry",
            self.telemetry.append, 10)
        self.create_subscription(
            PathHealth, "/path_health", self.path_health.append, 10)
        self.create_subscription(
            CmdVelFinal, "/cmd_vel_final", self.final_commands.append, 10)
        self.create_subscription(
            String, "/controller/status", self._on_controller_status, 10)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(
            NavPath, "/route_executor/active_chunk_path", self._on_active_chunk, 10)
        self.guard = self.create_publisher(
            BatteryMissionGuard, "/smoke/battery_mission_guard", 10)
        self.set_patrol = self.create_client(
            SetPatrolMissionLL, "/route_executor/set_patrol_mission_ll")
        self.state = self.create_client(
            GetPatrolMissionState, "/route_executor/get_patrol_mission_state")
        self.route_state = self.create_client(
            GetRouteMissionState, "/route_executor/get_route_mission_state")
        self.cancel = self.create_client(
            CancelPatrolMission, "/route_executor/cancel_patrol_mission")
        self.startup = subscribe_navigation_startup(self)

    def _on_plan(self, message):
        if self.capture_plan_topology:
            self.plans.append(_path_record(message))

    def _on_event(self, message):
        self.events.append(message)
        self.event_receipts.append({
            "message": message,
            "received_monotonic_s": time.monotonic(),
        })

    def _on_controller_status(self, message):
        received = time.monotonic()
        try:
            payload = json.loads(str(message.data))
            command = payload.get("command", {})
            self.controller_status.append({
                "received_monotonic_s": received,
                "valid": math.isfinite(float(command.get("requested_linear_x_mps", 0.0))),
                "requested_linear_x_mps": float(command.get("requested_linear_x_mps", 0.0)),
                "source": payload.get("source", ""),
            })
        except (TypeError, ValueError, json.JSONDecodeError):
            self.controller_status.append({
                "received_monotonic_s": received,
                "valid": False,
            })

    def _on_active_chunk(self, message):
        if self.capture_plan_topology:
            self.active_chunks.append(_path_record(message))

    def begin_plan_topology_capture(self):
        self.plans.clear()
        self.active_chunks.clear()
        self.controller_status.clear()
        self.event_receipts.clear()
        self.capture_plan_topology = True

    def publish_guard(self):
        now = time.monotonic()
        if now - self.last_guard_publish < 0.1:
            return
        message = BatteryMissionGuard()
        message.stamp = self.get_clock().now().to_msg()
        message.ready = True
        message.fresh = True
        message.return_home_recommended = self.guard_low
        message.state = "RETURN_HOME" if self.guard_low else "WATCHING"
        message.operator_soc_pct = 20.0 if self.guard_low else 80.0
        self.guard.publish(message)
        self.guard_publications += 1
        self.last_guard_publish = now


def ll_from_local(x, y):
    return (
        LAT + y / 111_320.0,
        LON + x / (111_320.0 * math.cos(math.radians(LAT))),
    )


def patrol_request(pose, *, home_at_origin):
    yaw = math.atan2(
        2.0 * pose.orientation.w * pose.orientation.z,
        1.0 - 2.0 * pose.orientation.z ** 2)
    origin_x, origin_y = pose.position.x, pose.position.y

    def world(forward, left):
        return (
            origin_x + forward * math.cos(yaw) - left * math.sin(yaw),
            origin_y + forward * math.sin(yaw) + left * math.cos(yaw),
        )

    # Use a broad loop rather than a collinear loop or a tight diamond.  Each
    # segment is about 19.8 m and each corner is roughly 90 degrees, giving the
    # Ackermann/Dubins model room for the 4 m minimum turning radius while
    # keeping the battery-return path short enough for this smoke.
    loop_xy = [
        world(6.0, 0.0), world(20.0, -14.0),
        world(34.0, 0.0), world(20.0, 14.0),
    ]
    loop_ll = [ll_from_local(x, y) for x, y in loop_xy]
    home_xy = (origin_x, origin_y) if home_at_origin else loop_xy[-1]
    home_lat, home_lon = ll_from_local(*home_xy)
    request = SetPatrolMissionLL.Request()
    request.loop_lats = [value[0] for value in loop_ll]
    request.loop_lons = [value[1] for value in loop_ll]
    request.loop_waypoint_action_jsons = ["" for _ in loop_ll]
    request.home_lat, request.home_lon = home_lat, home_lon
    request.home_yaw_deg = math.degrees(yaw)
    request.depart_entry_loop_index = 0
    # Zero means "use the coordinator's productive defaults": 35/120/5.
    request.leg_spacing_m = 0.0
    request.chunk_span_m = 0.0
    request.chunk_max_waypoints = 0
    return request


def main():
    rclpy.init()
    node = Smoke()
    runtime = SmokeRuntime(
        node, "patrol-battery-return",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "patrol_battery_probe.json",
        global_timeout_s=220.0)
    success = False
    failure = None
    poller = AsyncServicePoller(
        node.state, GetPatrolMissionState.Request,
        interval_s=0.25, response_timeout_s=5.0)
    route_poller = AsyncServicePoller(
        node.route_state, GetRouteMissionState.Request,
        interval_s=0.25, response_timeout_s=5.0)
    join_odom_start = 0

    def stamp_s(stamp):
        return stamp.sec + stamp.nanosec * 1.0e-9

    def pose_summary(start=0):
        if len(node.odom) <= start:
            return {"messages": 0}
        points = [message.pose.pose.position for message in node.odom[start:]]
        travelled = sum(math.hypot(right.x - left.x, right.y - left.y)
                        for left, right in zip(points, points[1:]))
        return {
            "messages": len(points),
            "initial": {"x": points[0].x, "y": points[0].y},
            "final": {"x": points[-1].x, "y": points[-1].y},
            "distance_travelled_m": travelled,
        }

    def event_history():
        receipts = {
            id(item["message"]): item["received_monotonic_s"]
            for item in node.event_receipts
        }
        return [{
            "timestamp_s": stamp_s(event.stamp),
            "event_id": event.event_id,
            "component": event.component,
            "code": event.code,
            "message": event.message,
            "details": _event_details(
                {entry.key: entry.value for entry in event.details}),
            "received_monotonic_s": receipts.get(id(event)),
        } for event in node.events]

    def odometry_records():
        return [_odom_record(message) for message in node.odom]

    def diagnostic_evidence():
        route = route_poller.latest
        telemetry = node.telemetry[-1] if node.telemetry else None
        health = node.path_health[-1] if node.path_health else None
        command = node.final_commands[-1] if node.final_commands else None
        return {
            "phase_history": node.phase_history,
            "guard_publications": node.guard_publications,
            "events": event_history(),
            "odometry": pose_summary(),
            "join_loop_odometry": pose_summary(join_odom_start),
            "route_executor": None if route is None else {
                "mission_id": route.mission_id,
                "status": route.status,
                "active": route.active,
                "current_target_index": route.current_target_index,
                "distance_to_target_m": route.distance_to_target_m,
                "blocked_state": route.blocked_state,
                "blocked_reason_code": route.blocked_reason_code,
                "blocked_retry_attempt": route.blocked_retry_attempt,
            },
            "nav_telemetry": None if telemetry is None else {
                "goal_active": telemetry.goal_active,
                "nav_result_status": telemetry.nav_result_status,
                "nav_result_text": telemetry.nav_result_text,
                "nav_result_event_id": telemetry.nav_result_event_id,
                "failure_code": telemetry.failure_code,
                "cmd_vel_safe_fresh": telemetry.cmd_vel_safe_fresh,
                "cmd_vel_linear_x": telemetry.cmd_vel_linear_x,
                "cmd_vel_angular_z": telemetry.cmd_vel_angular_z,
                "collision_stop_active": telemetry.collision_stop_active,
            },
            "last_cmd_vel_final": None if command is None else {
                "linear_x": command.twist.linear.x,
                "angular_z": command.twist.angular.z,
                "brake_pct": command.brake_pct,
                "source": command.source,
            },
            "path_health": None if health is None else {
                "timestamp_s": stamp_s(health.stamp),
                "state": health.state,
                "reason": health.reason,
                "costmap_age_s": health.costmap_age_s,
                "cross_track_error_m": health.cross_track_error_m,
                "max_cost": health.max_cost,
            },
            "path_health_history": [{
                "timestamp_s": stamp_s(item.stamp),
                "state": item.state,
                "reason": item.reason,
                "costmap_age_s": item.costmap_age_s,
                "cross_track_error_m": item.cross_track_error_m,
            } for item in node.path_health],
            "planner_path_topology": plan_topology_evidence(
                node.plans,
                node.active_chunks,
                events=event_history(),
                odometry=odometry_records(),
                controller_status=node.controller_status,
            ),
            "controller_status_samples": node.controller_status,
            "state_poller": poller.evidence(),
            "route_state_poller": route_poller.evidence(),
        }

    def stimulate():
        node.publish_guard()
        poller.poll()
        route_poller.poll()
        if poller.latest is not None:
            phase = poller.latest.status
            if not node.phase_history or node.phase_history[-1] != phase:
                node.phase_history.append(phase)

    def state_is(*phases):
        return poller.latest is not None and poller.latest.status in phases

    def settled_at_home():
        state = poller.latest
        route = route_poller.latest
        telemetry = node.telemetry[-1] if node.telemetry else None
        command = node.final_commands[-1] if node.final_commands else None
        return bool(
            state is not None
            and state.status == "AT_HOME"
            and state.low_battery_active
            and not state.active
            and route is not None
            and not route.active
            and telemetry is not None
            and not telemetry.goal_active
            and command is not None
            and command.twist.linear.x == 0.0
            and command.twist.angular.z == 0.0
        )

    try:
        runtime.wait(
            "patrol startup", lambda: (
                node.startup.active
                and node.set_patrol.service_is_ready()
                and node.state.service_is_ready()
                and node.cancel.service_is_ready()
                and has_increasing_stamps(node.odom)
                and finite_odometry(node.odom[-1])
                and node.guard.get_subscription_count() >= 1),
            45.0, stimulate=stimulate,
            observe=lambda: {
                "startup": node.startup.snapshot(),
                "odom_messages": len(node.odom),
                "guard_subscribers": node.guard.get_subscription_count(),
                "services": [node.set_patrol.service_is_ready(),
                             node.state.service_is_ready(),
                             node.cancel.service_is_ready()],
            })

        # A mission accepted while the guard already requests return must stay
        # parked at HOME and must never dispatch its departure/loop.
        node.guard_low = True
        runtime.wait(
            "low guard delivered", lambda: node.guard_publications >= 3,
            5.0, stimulate=stimulate)
        request = patrol_request(node.odom[-1].pose.pose, home_at_origin=True)
        accepted = runtime.call(
            "set low-battery patrol", node.set_patrol, request, timeout_s=8.0)
        if not accepted.ok:
            raise RuntimeError(f"low-battery patrol rejected: {accepted.error}")
        runtime.wait(
            "patrol held at HOME",
            lambda: state_is("AT_HOME") and poller.latest.low_battery_active
            and not poller.latest.active,
            15.0, stimulate=stimulate,
            observe=lambda: None if poller.latest is None else {
                "status": poller.latest.status,
                "active": poller.latest.active,
                "low_battery_active": poller.latest.low_battery_active,
            })
        dispatched_before_cancel = sum(
            event.code == "PATROL_PHASE_DISPATCHED" for event in node.events)
        if dispatched_before_cancel:
            raise RuntimeError("low-battery patrol dispatched motion from HOME")
        cancelled = runtime.call(
            "cancel parked patrol", node.cancel,
            CancelPatrolMission.Request(), timeout_s=8.0)
        if not cancelled.ok:
            raise RuntimeError(f"parked patrol cancellation failed: {cancelled.error}")

        # Start normally, enter the loop, then latch a battery return. Recovery
        # samples remain false only after the return is already committed.
        node.guard_low = False
        baseline_publications = node.guard_publications
        runtime.wait(
            "healthy guard delivered",
            lambda: node.guard_publications >= baseline_publications + 3,
            5.0, stimulate=stimulate)
        node.begin_plan_topology_capture()
        accepted = runtime.call(
            "set active patrol", node.set_patrol,
            # HOME at the spawn pose makes the selected return exit the next
            # checkpoint after entering PATROL.  The previous HOME=P3 setup
            # required traversing most of the loop before the coordinator
            # could cancel it and dispatch HOME, which made this functional
            # smoke timeout inside EXIT_LOOP.
            patrol_request(node.odom[-1].pose.pose, home_at_origin=True),
            timeout_s=8.0)
        if not accepted.ok:
            raise RuntimeError(f"active patrol rejected: {accepted.error}")
        join_odom_start = len(node.odom)
        runtime.wait(
            "patrol entered loop", lambda: state_is("PATROL"),
            70.0, stimulate=stimulate,
            observe=lambda: {"phase_history": node.phase_history})
        runtime.wait(
            "planner path observed in patrol",
            lambda: bool(plan_topology_evidence(node.plans, node.active_chunks)),
            10.0, stimulate=stimulate,
            observe=lambda: {
                "plans": len(node.plans),
                "active_chunks": len(node.active_chunks),
            })
        # Path geometry remains in the evidence bundle, but it is gated by
        # the dedicated navigation-quality smoke.  This smoke owns only the
        # battery/patrol state-machine contract.
        node.guard_low = True
        runtime.wait(
            "battery return requested",
            lambda: state_is("EXIT_LOOP", "RETURN_HOME", "AT_HOME")
            and poller.latest.low_battery_active,
            12.0, stimulate=stimulate,
            observe=lambda: {"phase_history": node.phase_history})
        node.guard_low = False
        runtime.wait(
            "latched return reached HOME and settled",
            settled_at_home,
            90.0, stimulate=stimulate,
            observe=lambda: {
                "phase_history": node.phase_history,
                "return_requested": (False if poller.latest is None
                                     else poller.latest.return_home_requested),
                "return_active": (False if poller.latest is None
                                  else poller.latest.return_home_active),
                "route_active": (None if route_poller.latest is None
                                  else route_poller.latest.active),
                "goal_active": (None if not node.telemetry
                                 else node.telemetry[-1].goal_active),
                "last_cmd_vel_final": (
                    None if not node.final_commands else {
                        "linear_x": node.final_commands[-1].twist.linear.x,
                        "angular_z": node.final_commands[-1].twist.angular.z,
                        "brake_pct": node.final_commands[-1].brake_pct,
                    }),
            })
        required = {"PATROL", "EXIT_LOOP", "RETURN_HOME", "AT_HOME"}
        if not required.issubset(set(node.phase_history)):
            raise RuntimeError(
                f"incomplete battery return phases: {node.phase_history}")
        success = True
        runtime.report.evidence = diagnostic_evidence()
    except Exception as exc:
        failure = exc
        runtime.report.evidence = diagnostic_evidence()
        runtime.report.evidence["startup"] = node.startup.snapshot()
    finally:
        runtime.finish(success, error=failure)
        node.destroy_node()
        rclpy.shutdown()
    if failure is not None:
        print(f"Patrol battery smoke failed: {failure}", file=sys.stderr)
        return 1
    print("Patrol battery return smoke passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
