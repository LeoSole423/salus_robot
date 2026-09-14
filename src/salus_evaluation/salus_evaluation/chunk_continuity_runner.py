"""Observe productive route_executor continuity across a wide-turn boundary."""

from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from salus_interfaces.msg import CmdVelFinal, NavEvent, NavTelemetry
from salus_interfaces.srv import (
    CancelRouteMission, GetRouteMissionState, SetRouteMissionLL,
)
from std_msgs.msg import String

from salus_navigation.route_geometry import path_geometry_metrics

from .artifacts import write_artifacts
from .geometry_quality import quality_metrics
from .metrics import angle_delta


CURRENT_POLICY = "terminal_incoming"
BASE_LAT = -31.4858037
BASE_LON = -64.2410570
TURN_RADIUS_M = 8.0
BOUNDARY_TURN_DEG = 30.0
ROUTE_SPACING_M = 1.0
ROUTE_CHUNK_SPAN_M = 100.0
ROUTE_CHUNK_MAX_WAYPOINTS = 20


def _yaw(quaternion):
    """Extract planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _stamp(message):
    """Return a ROS message timestamp in seconds."""
    stamp = message.header.stamp if hasattr(message, "header") else message.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _point_path(message):
    """Convert a nav path to finite XY points."""
    return tuple(
        (float(item.pose.position.x), float(item.pose.position.y))
        for item in message.poses
    )


def _body_to_map(origin, forward, lateral):
    """Place a body-frame point at the observed map pose."""
    x, y, yaw = origin
    return (
        x + forward * math.cos(yaw) - lateral * math.sin(yaw),
        y + forward * math.sin(yaw) + lateral * math.cos(yaw),
    )


def _wide_turn_route(origin):
    """Construct a broad ninety-degree left-turn route."""
    # The first chunk ends at the 30-degree checkpoint. Its successor segment
    # is still inside the broad turn, making the boundary causal rather than a
    # separate straight-line goal experiment.
    center_forward = 1.5
    points = []
    for degrees in (-90.0, -60.0, -30.0, 0.0):
        theta = math.radians(degrees)
        points.append(_body_to_map(
            origin,
            center_forward + TURN_RADIUS_M * math.cos(theta),
            TURN_RADIUS_M + TURN_RADIUS_M * math.sin(theta),
        ))
    return points


def _boundary_corner_route(origin):
    """Construct a hard logical corner for the chunk-boundary case."""
    return [
        _body_to_map(origin, 2.0, 0.0),
        _body_to_map(origin, 8.0, 0.0),
        _body_to_map(origin, 8.0, 8.0),
    ]


def _route_for_scenario(origin, scenario):
    """Select only the named evaluation fixture, never a production policy."""
    if "boundary_corner_90" in scenario:
        return _boundary_corner_route(origin)
    return _wide_turn_route(origin)


def _local_to_ll(point):
    """Use the simulation datum conversion used by the existing route smoke."""
    x, y = point
    return (
        BASE_LAT + y / 111320.0,
        BASE_LON + x / (111320.0 * math.cos(math.radians(BASE_LAT))),
    )


def _finite_pose(message):
    """Return a serializable pose sample."""
    pose = message.pose.pose
    values = (pose.position.x, pose.position.y, _yaw(pose.orientation))
    if not all(math.isfinite(float(value)) for value in values):
        return None
    return {
        "stamp_s": _stamp(message),
        "x_m": float(values[0]),
        "y_m": float(values[1]),
        "yaw_rad": float(values[2]),
    }


def _details(event):
    """Decode NavEvent key/value details, including JSON values."""
    result = {item.key: item.value for item in event.details}
    for key in ("input_indices", "synthetic_offsets", "yaws_deg", "poses_xy",
                "robot_xy", "goal_generation"):
        if key not in result:
            continue
        try:
            result[key] = json.loads(result[key])
        except (TypeError, ValueError):
            pass
    return result


def _event_record(event):
    """Return the event fields needed for causal correlation."""
    return {
        "stamp_s": _stamp(event),
        "event_id": int(event.event_id),
        "severity": int(event.severity),
        "component": str(event.component),
        "code": str(event.code),
        "message": str(event.message),
        "details": _details(event),
    }


def _segment_heading(points, at_end=False):
    """Get a realized polyline heading at its incoming or outgoing edge."""
    if len(points) < 2:
        return None
    first, second = (points[-2], points[-1]) if at_end else (points[0], points[1])
    return math.atan2(second[1] - first[1], second[0] - first[0])


def _orientation(first, second, third):
    """Return the signed area used by the segment crossing test."""
    return ((second[0] - first[0]) * (third[1] - first[1])
            - (second[1] - first[1]) * (third[0] - first[0]))


def _on_segment(first, point, last):
    """Return whether a collinear point is inside a segment."""
    return (min(first[0], last[0]) - 1e-9 <= point[0] <= max(first[0], last[0]) + 1e-9
            and min(first[1], last[1]) - 1e-9 <= point[1] <= max(first[1], last[1]) + 1e-9)


def _segments_intersect(first, second):
    """Return whether two closed XY segments intersect."""
    a, b = first
    c, d = second
    values = (_orientation(a, b, c), _orientation(a, b, d),
              _orientation(c, d, a), _orientation(c, d, b))
    if ((values[0] > 1e-9 and values[1] < -1e-9
         or values[0] < -1e-9 and values[1] > 1e-9)
            and (values[2] > 1e-9 and values[3] < -1e-9
                 or values[2] < -1e-9 and values[3] > 1e-9)):
        return True
    return any(abs(value) <= 1e-9 and _on_segment(start, middle, end)
               for value, start, middle, end in (
                   (values[0], a, c, b), (values[1], a, d, b),
                   (values[2], c, a, d), (values[3], c, b, d)))


def _polyline_length(points):
    """Return the length of a finite XY polyline."""
    return sum(math.hypot(end[0] - start[0], end[1] - start[1])
               for start, end in zip(points, points[1:]))


def _self_intersections(points):
    """Count non-adjacent self-intersections within one plan."""
    segments = tuple(zip(points, points[1:]))
    return sum(
        _segments_intersect(first, second)
        for index, first in enumerate(segments)
        for second in segments[index + 2:]
        if not set(first).intersection(second)
    )


def _cross_intersections(first, second):
    """Count crossings between plan A and plan B, excluding shared joins."""
    first_segments = tuple(zip(first, first[1:]))
    second_segments = tuple(zip(second, second[1:]))
    return sum(
        _segments_intersect(left, right)
        for left in first_segments for right in second_segments
        if not set(left).intersection(right)
    )


def _transition_metrics(first_plan, second_plan, reference):
    """Measure path geometry and heading continuity at a chunk transition."""
    if not first_plan or not second_plan:
        return {"available": False, "reason": "both chunk plans were not observed"}
    geometry_a = path_geometry_metrics(first_plan, reference)
    geometry_b = path_geometry_metrics(second_plan, reference)
    incoming = _segment_heading(first_plan, at_end=True)
    outgoing = _segment_heading(second_plan)
    heading_change = abs(angle_delta(outgoing, incoming))
    boundary_distance = math.hypot(
        second_plan[0][0] - first_plan[-1][0],
        second_plan[0][1] - first_plan[-1][1],
    )
    return {
        "available": True,
        "length_plan_A_m": geometry_a.length_m,
        "length_plan_B_m": geometry_b.length_m,
        "length_total_executable_estimated_m": (
            geometry_a.length_m + geometry_b.length_m
        ),
        "self_intersections_plan_A": geometry_a.self_intersections,
        "self_intersections_plan_B": geometry_b.self_intersections,
        "cross_intersections_A_B": _cross_intersections(first_plan, second_plan),
        "self_intersections": None,
        "self_intersections_note": "not computed across independent plans",
        "heading_final_A_rad": incoming,
        "heading_initial_B_rad": outgoing,
        "distance_final_A_to_initial_B_m": boundary_distance,
        "boundary_angular_discontinuity_rad": heading_change,
        "incoming_heading_rad": incoming,
        "outgoing_heading_rad": outgoing,
        "heading_change_rad": heading_change,
        "heading_change_deg": math.degrees(heading_change),
    }


def _nearest_odom(samples, stamp_s):
    """Return the global odometry sample closest to a ROS timestamp."""
    if not samples:
        return None
    return min(samples, key=lambda item: abs(item["stamp_s"] - stamp_s))


def _goal_generation(record):
    """Return a goal generation from an event record, if present."""
    value = record.get("details", {}).get("goal_generation")
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _chunk_windows(dispatches, events, plans, odometry, reference):
    """Associate every chunk dispatch with its goal and all plans in that window."""
    windows = []
    result_codes = {
        "GOAL_RESULT_SUCCEEDED", "GOAL_RESULT_ABORTED", "GOAL_CANCELLED",
    }
    for index, dispatch in enumerate(dispatches):
        start_s = dispatch["stamp_s"]
        next_dispatch_s = (
            dispatches[index + 1]["stamp_s"]
            if index + 1 < len(dispatches) else float("inf")
        )
        accepted = next((event for event in events
                         if event["code"] == "GOAL_ACCEPTED"
                         and start_s <= event["stamp_s"] < next_dispatch_s), None)
        generation = None if accepted is None else _goal_generation(accepted)
        result = next((event for event in events
                       if event["code"] in result_codes
                       and event["stamp_s"] >= start_s
                       and event["stamp_s"] < next_dispatch_s
                       and (generation is None
                            or _goal_generation(event) == generation)), None)
        end_s = min(next_dispatch_s, result["stamp_s"] if result else next_dispatch_s)
        window_plans = [item for item in plans
                        if start_s <= item["stamp_s"] < end_s]
        plan_records = []
        for plan_index, item in enumerate(window_plans):
            plan_records.append({
                "plan_index": plan_index,
                "stamp_s": item["stamp_s"],
                "points": item["points"],
                "odometry_global_near_plan": _nearest_odom(
                    odometry, item["stamp_s"]
                ),
                "metrics": _plan_metrics(item["points"], reference),
            })
        windows.append({
            "chunk_id": dispatch.get("chunk_id"),
            "dispatch_stamp_s": start_s,
            "dispatch": dispatch,
            "goal_accepted": accepted,
            "goal_generation": generation,
            "goal_result": result,
            "result_stamp_s": None if result is None else result["stamp_s"],
            "odometry_global_near_dispatch": _nearest_odom(odometry, start_s),
            "odometry_global_near_result": (
                None if result is None
                else _nearest_odom(odometry, result["stamp_s"])
            ),
            "plans": plan_records,
        })
    return windows


def _plan_metrics(points, reference):
    """Return geometry metrics for one independent Nav2 plan."""
    if not points:
        return {"available": False, "reason": "empty plan"}
    geometry = path_geometry_metrics(points, reference or points)
    return {
        "available": True,
        "length_m": geometry.length_m,
        "direct_distance_m": geometry.direct_distance_m,
        "detour_ratio": geometry.detour_ratio,
        "max_deviation_m": geometry.max_deviation_m,
        "self_intersections": geometry.self_intersections,
        "geometry_quality": quality_metrics(points, reference or points),
    }


class ChunkContinuityRunner(Node):
    """Drive one route through the public route_executor service and observe it."""

    def __init__(self):
        super().__init__(
            "navigation_chunk_continuity",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.declare_parameter("output_dir", "")
        self.declare_parameter("chunk_policy", CURRENT_POLICY)
        self.declare_parameter("scenario", "")
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.policy = str(self.get_parameter("chunk_policy").value)
        self.scenario = str(self.get_parameter("scenario").value)
        if not self.output_dir:
            raise ValueError("output_dir is required")
        if self.policy != CURRENT_POLICY:
            raise ValueError("Track 3 runner only accepts CURRENT/terminal_incoming")

        self.odom = []
        self.raw_odom = []
        self.mission_paths = []
        self.active_chunks = []
        self.plans = []
        self.events = []
        self.telemetry = []
        self.controller_status = []
        self.final_commands = []
        self.set_route = self.create_client(
            SetRouteMissionLL, "/route_executor/set_route_mission_ll"
        )
        self.get_state = self.create_client(
            GetRouteMissionState, "/route_executor/get_route_mission_state"
        )
        self.cancel_route = self.create_client(
            CancelRouteMission, "/route_executor/cancel_route_mission"
        )
        self.reference = None
        self.route_request = None
        self.dispatches = []
        self.started_at = None
        self.last_state = None

        self.create_subscription(Odometry, "/odometry/global", self._odom, 50)
        self.create_subscription(Odometry, "/odom_raw", self._raw_odom, 50)
        self.create_subscription(NavPath, "/route_executor/mission_path",
                                 self._mission_path, 10)
        self.create_subscription(NavPath, "/route_executor/active_chunk_path",
                                 self._active_chunk, 10)
        self.create_subscription(NavPath, "/plan", self._plan, 20)
        self.create_subscription(NavEvent, "/nav_command_server/events",
                                 self._event, 50)
        self.create_subscription(NavTelemetry, "/nav_command_server/telemetry",
                                 self.telemetry.append, 20)
        self.create_subscription(String, "/controller/status", self._status, 20)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final",
                                 self.final_commands.append, 50)

    def _odom(self, message):
        sample = _finite_pose(message)
        if sample is not None:
            self.odom.append(sample)

    def _raw_odom(self, message):
        sample = _finite_pose(message)
        if sample is not None:
            self.raw_odom.append(sample)

    def _mission_path(self, message):
        points = _point_path(message)
        if points:
            self.mission_paths.append({"stamp_s": _stamp(message), "points": points})

    def _active_chunk(self, message):
        points = _point_path(message)
        if points:
            self.active_chunks.append({"stamp_s": _stamp(message), "points": points})

    def _plan(self, message):
        points = _point_path(message)
        if points:
            self.plans.append({"stamp_s": _stamp(message), "points": points})

    def _event(self, message):
        record = _event_record(message)
        self.events.append(record)
        if record["code"] == "ROUTE_CHUNK_DISPATCHED":
            details = record["details"]
            robot_xy = details.get("robot_xy")
            robot_pose = None
            if isinstance(robot_xy, list) and len(robot_xy) == 2:
                robot_pose = _nearest_odom(self.odom, record["stamp_s"])
            self.dispatches.append({
                **record,
                "chunk_id": details.get("chunk_id", record["event_id"]),
                "robot_xy_from_event": robot_xy,
                "robot_pose_at_dispatch": robot_pose,
                "yaws_deg": details.get("yaws_deg"),
                "input_indices": details.get("input_indices"),
                "synthetic_offsets": details.get("synthetic_offsets"),
                "poses_xy": details.get("poses_xy"),
            })

    def _status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict):
            self.controller_status.append({
                "stamp_s": self.get_clock().now().nanoseconds / 1e9,
                "payload": payload,
            })

    def _spin_wait(self, predicate, timeout_s, label):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            rclpy.spin_once(self, timeout_sec=0.1)

    def _call(self, client, request, timeout_s=10.0):
        self._spin_wait(client.service_is_ready, timeout_s,
                        f"service unavailable: {client.srv_name}")
        future = client.call_async(request)
        self._spin_wait(future.done, timeout_s,
                        f"service call timed out: {client.srv_name}")
        response = future.result()
        if response is None:
            raise RuntimeError(f"empty response: {client.srv_name}")
        return response

    def _state(self):
        return self._call(self.get_state, GetRouteMissionState.Request())

    def _request(self, points):
        request = SetRouteMissionLL.Request()
        converted = [_local_to_ll(point) for point in points]
        request.lats = [item[0] for item in converted]
        request.lons = [item[1] for item in converted]
        request.yaws_deg = [float("nan")] * len(points)
        request.waypoint_action_jsons = []
        request.waypoint_roles = []
        request.loop = False
        request.leg_spacing_m = ROUTE_SPACING_M
        request.chunk_span_m = ROUTE_CHUNK_SPAN_M
        request.chunk_max_waypoints = ROUTE_CHUNK_MAX_WAYPOINTS
        return request

    def _wait_for_transition(self, timeout_s=105.0):
        def ready():
            if len(self.dispatches) >= 2:
                return True
            try:
                self.last_state = self._state()
            except (RuntimeError, TimeoutError):
                return False
            return False

        self._spin_wait(ready, timeout_s,
                        "productive route_executor did not dispatch chunk B")
        self._spin_wait(
            lambda: any(
                item["stamp_s"] >= self.dispatches[1]["stamp_s"]
                for item in self.plans
            ),
            15.0,
            "did not observe /plan for chunk B",
        )

    def run(self):
        self._spin_wait(lambda: bool(self.odom), 30.0,
                        "global odometry unavailable")
        origin_sample = self.odom[-1]
        origin = (origin_sample["x_m"], origin_sample["y_m"], origin_sample["yaw_rad"])
        self.reference = _route_for_scenario(origin, self.scenario)
        self.route_request = self._request(self.reference)
        self.started_at = self.get_clock().now().nanoseconds / 1e9
        response = self._call(self.set_route, self.route_request)
        if not response.ok:
            raise RuntimeError(f"route_executor rejected mission: {response.error}")
        self._wait_for_transition()
        # Allow callbacks after dispatch B to settle, then cancel through the
        # public service so no evaluator-owned action is involved.
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        cancel = self._call(self.cancel_route, CancelRouteMission.Request())
        if not cancel.ok:
            raise RuntimeError(f"route cancellation failed: {cancel.error}")
        self._spin_wait(
            lambda: self._state().status == "CANCELLED", 8.0,
            "route_executor did not reach CANCELLED after observation",
        )
        return self._summary(origin)

    def _summary(self, origin):
        mission_path = self.mission_paths[-1]["points"] if self.mission_paths else ()
        active_paths = [item["points"] for item in self.active_chunks]
        route_reference = tuple(mission_path) or tuple(self.reference)
        windows = _chunk_windows(
            self.dispatches, self.events, self.plans, self.odom, route_reference
        )
        geometry_quality = [
            {
                "plan_index": plan["plan_index"],
                "stamp_s": plan["stamp_s"],
                **plan["metrics"]["geometry_quality"],
            }
            for window in windows for plan in window["plans"]
        ]
        first_window_plans = windows[0]["plans"] if windows else []
        second_window_plans = windows[1]["plans"] if len(windows) > 1 else []
        # Use final A and first B only for boundary continuity; all plans stay
        # represented in chunk_windows.
        first_plan = first_window_plans[-1]["points"] if first_window_plans else ()
        second_plan = second_window_plans[0]["points"] if second_window_plans else ()
        transition = _transition_metrics(first_plan, second_plan, route_reference)
        dispatch_b = self.dispatches[1] if len(self.dispatches) > 1 else None
        terminal_a = []
        if windows and windows[0]["goal_result"] is not None:
            terminal_a = [windows[0]["goal_result"]]
        route_events = [item for item in self.events
                        if item["code"].startswith("ROUTE_")]
        summary = {
            "schema_version": 3,
            "reason": "productive_route_executor_chunk_continuity",
            "conclusion": (
                "OBSERVED_PARTIAL" if transition.get("available") and terminal_a
                else "NOT_REPRODUCED/INCONCLUSIVE"
            ),
            "classification": "INCONCLUSIVE",
            "terminal_status": None,
            "errors": [],
            "route_executor": {
                "service": "/route_executor/set_route_mission_ll",
                "used_productive_service": True,
                "manual_nav2_action_used": False,
                "route_request": {
                    "input_waypoints": len(self.reference),
                    "loop": False,
                    "leg_spacing_m": ROUTE_SPACING_M,
                    "chunk_span_m": ROUTE_CHUNK_SPAN_M,
                    "chunk_max_waypoints": ROUTE_CHUNK_MAX_WAYPOINTS,
                    "yaws_are_automatic_nan": True,
                },
                "origin_pose": origin,
                "mission_path": mission_path,
                "active_chunk_path_snapshots": active_paths,
                "dispatched_yaws_deg": [item.get("yaws_deg") for item in self.dispatches],
                "dispatches": self.dispatches,
                "route_events": route_events,
                "terminal_chunk_a_events": terminal_a,
                "chunk_b_dispatch": dispatch_b,
                "chunk_windows": windows,
            },
            "transition": transition,
            "plans": self.plans,
            "geometry_quality": geometry_quality,
            "observed_metrics": {
                "active_chunk_count": len(active_paths),
                "dispatch_count": len(self.dispatches),
                "plan_count": len(self.plans),
                "odom_samples": len(self.odom),
                "final_auto_commands": sum(
                    int(item.source) == CmdVelFinal.SOURCE_AUTO
                    for item in self.final_commands
                ),
                "final_brake_samples": sum(
                    int(item.brake_pct) > 0 for item in self.final_commands
                ),
            },
        }
        manifest = {
            "schema_version": 3,
            "mode": "chunk_continuity",
            "policy": self.policy,
            "scenario": self.scenario or "wide_90deg_turn_boundary_inside",
            "route_executor_service": "/route_executor/set_route_mission_ll",
            "manual_navigate_through_poses_action": False,
            "geometry": {
                "turn_radius_m": (
                    None if "boundary_corner_90" in self.scenario else TURN_RADIUS_M
                ),
                "turn_angle_deg": 90.0,
                "boundary_after_turn_deg": (
                    0.0 if "boundary_corner_90" in self.scenario
                    else BOUNDARY_TURN_DEG
                ),
                "boundary_inside_broad_turn": "boundary_corner_90" not in self.scenario,
                "reference_points": self.reference,
            },
            "topics": [
                "/route_executor/mission_path",
                "/route_executor/active_chunk_path",
                "/nav_command_server/events",
                "/plan",
                "/odometry/global",
                "/odom_raw",
                "/cmd_vel_final",
            ],
            "streams": [
                "odometry_global", "odometry_raw", "route_events",
                "mission_path", "active_chunk_path", "plans",
            ],
        }
        write_artifacts(self.output_dir, manifest, summary, {
            "odometry_global": self.odom,
            "odometry_raw": self.raw_odom,
            "route_events": self.events,
            "mission_path": self.mission_paths,
            "active_chunk_path": self.active_chunks,
            "plans": self.plans,
        })
        return summary


def main():
    rclpy.init()
    node = ChunkContinuityRunner()
    try:
        node.run()
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        write_artifacts(
            node.output_dir,
            {
                "schema_version": 3,
                "mode": "chunk_continuity",
                "policy": node.policy,
                "route_executor_service": "/route_executor/set_route_mission_ll",
                "manual_navigate_through_poses_action": False,
            },
            {
                "schema_version": 3,
                "reason": "setup_or_observation_failure",
                "conclusion": "NOT_REPRODUCED/INCONCLUSIVE",
                "errors": [str(exc)],
                "route_executor": {
                    "used_productive_service": True,
                    "manual_nav2_action_used": False,
                    "dispatches": node.dispatches,
                    "events": node.events,
                },
            },
            {
                "odometry_global": node.odom,
                "odometry_raw": node.raw_odom,
                "route_events": node.events,
                "mission_path": node.mission_paths,
                "active_chunk_path": node.active_chunks,
                "plans": node.plans,
            },
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
