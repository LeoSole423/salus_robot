"""ROS adapter for the small, testable route-domain policies.

The node owns mission lifecycle only.  It never sends a velocity command or a
Nav2 action: every waypoint crosses nav_command_server.
"""
from __future__ import annotations

import threading
import uuid

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from robot_localization.srv import FromLL
from salus_interfaces.msg import NavTelemetry
from salus_interfaces.srv import (
    BrakeNav, CancelNavGoal, CancelRouteMission, GetRouteMissionState,
    SetNavGoalLL, SetRouteMissionLL,
)

from .route_anchor import select_anchor
from .route_chunker import build_chunk, next_start
from .route_model import RouteMission, RoutePhase, RouteWaypoint
from .route_preparation import prepare, validate_inputs
from .route_progress import project
from .route_state_machine import transition


class RouteExecutorNode(Node):
    """Keep a pending preparation separate from the active mission."""

    def __init__(self) -> None:
        super().__init__("route_executor")
        self.declare_parameter("waypoint_reached_tolerance_m", 1.2)
        self.declare_parameter("fromll_timeout_s", 2.0)
        self._lock = threading.RLock()
        self._mission = RouteMission()
        self._preparation = None
        self._preparation_epoch = 0
        self._pose = None
        self._chunk = None
        self._target_offset = 0
        self._goal_epoch = 0
        self._last_result_event_id = -1
        self._set_goal = self.create_client(SetNavGoalLL, "/nav_command_server/set_goal_ll")
        self._cancel_goal = self.create_client(CancelNavGoal, "/nav_command_server/cancel_goal")
        self._brake = self.create_client(BrakeNav, "/nav_command_server/brake")
        self._fromll = [
            self.create_client(FromLL, "/fromLL"),
            self.create_client(FromLL, "/navsat_transform/fromLL"),
        ]
        self.create_subscription(Odometry, "/odometry/global", self._on_pose, 10)
        self.create_subscription(NavTelemetry, "/nav_command_server/telemetry", self._on_telemetry, 10)
        self._mission_path = self.create_publisher(Path, "/route_executor/mission_path", 10)
        self._chunk_path = self.create_publisher(Path, "/route_executor/active_chunk_path", 10)
        self.create_service(SetRouteMissionLL, "/route_executor/set_route_mission_ll", self._set)
        self.create_service(CancelRouteMission, "/route_executor/cancel_route_mission", self._cancel)
        self.create_service(GetRouteMissionState, "/route_executor/get_route_mission_state", self._state)
        self.create_timer(0.1, self._tick_preparation)

    def _on_pose(self, message: Odometry) -> None:
        self._pose = message.pose.pose.position

    def _set(self, request, response):
        lats, lons, yaws = list(request.lats), list(request.lons), list(request.yaws_deg)
        actions, roles = list(request.waypoint_action_jsons), list(request.waypoint_roles)
        error = validate_inputs(lats, lons, yaws, actions, roles)
        response.input_waypoint_count = len(lats)
        if error:
            response.ok, response.error = False, error
            return response
        raw = tuple(
            RouteWaypoint(float(lat), float(lon), float(yaw), index, True,
                          (actions or [""] * len(lats))[index],
                          (roles or ["normal"] * len(lats))[index])
            for index, (lat, lon, yaw) in enumerate(zip(lats, lons, yaws))
        )
        with self._lock:
            self._preparation_epoch += 1
            self._preparation = {
                "epoch": self._preparation_epoch, "raw": raw, "converted": [],
                "next": 0, "future": None, "request": request,
                "deadline": self._steady_now() + float(self.get_parameter("fromll_timeout_s").value),
            }
        # The old mission continues while conversion happens.  A later valid
        # preparation is the only thing allowed to replace it.
        response.ok, response.error = True, "preparing route conversion"
        response.expanded_waypoint_count = 0
        return response

    def _tick_preparation(self) -> None:
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
                    point = result.map_point
                    raw = job["raw"][job["next"]]
                    job["converted"].append(RouteWaypoint(
                        **{**raw.__dict__, "map_x": point.x, "map_y": point.y}
                    ))
                    job["next"] += 1
                    job["future"] = None
                    job["deadline"] = self._steady_now() + float(self.get_parameter("fromll_timeout_s").value)
                except Exception as exc:
                    self.get_logger().warning(f"route preparation rejected: fromLL failed: {exc}")
                    self._preparation = None
                    return
            elif future is not None and self._steady_now() > job["deadline"]:
                self.get_logger().warning("route preparation rejected: fromLL timed out")
                self._preparation = None
                return
            if job["future"] is not None:
                return
            if job["next"] == len(job["raw"]):
                self._activate_prepared(job)
                return
            client = next((item for item in self._fromll if item.service_is_ready()), None)
            if client is None:
                if self._steady_now() > job["deadline"]:
                    self.get_logger().warning("route preparation rejected: fromLL unavailable")
                    self._preparation = None
                return
            point = job["raw"][job["next"]]
            request = FromLL.Request()
            request.ll_point.latitude, request.ll_point.longitude = point.lat, point.lon
            request.ll_point.altitude = 0.0
            job["future"] = client.call_async(request)

    def _activate_prepared(self, job) -> None:
        request = job["request"]
        prepared = prepare(
            job["converted"], loop=bool(request.loop), input_count=len(job["raw"]),
            spacing_m=float(request.leg_spacing_m), chunk_span_m=float(request.chunk_span_m),
            chunk_max_waypoints=int(request.chunk_max_waypoints),
        )
        anchor = 0
        if self._pose is not None:
            anchor = select_anchor(
                prepared, self._pose.x, self._pose.y,
                float(self.get_parameter("waypoint_reached_tolerance_m").value),
            )
        prepared = type(prepared)(**{**prepared.__dict__, "anchor_input_index": anchor})
        mission = RouteMission(
            prepared=prepared, phase=RoutePhase.PREPARING,
            mission_id=str(uuid.uuid4()), target_index=anchor,
        )
        transition(mission, RoutePhase.ACTIVE)
        self._mission, self._preparation = mission, None
        self._dispatch()

    def _on_telemetry(self, message: NavTelemetry) -> None:
        with self._lock:
            if self._mission.phase != RoutePhase.ACTIVE:
                return
            if message.manual_enabled:
                self._pause("manual takeover")
                return
            if self._chunk is not None and self._pose is not None:
                self._mission.progress = project(self._chunk, self._pose.x, self._pose.y)
            if message.nav_result_event_id == self._last_result_event_id:
                return
            if message.goal_active or message.nav_result_text not in (
                    "succeeded", "aborted", "cancelled", "goal rejected"):
                return
            self._last_result_event_id = message.nav_result_event_id
            if message.nav_result_text == "succeeded":
                self._advance()
            elif message.nav_result_text == "goal rejected":
                self._pause("NAV_GOAL_REJECTED: NavigateToPose goal rejected")
            else:
                self._pause(f"navigation {message.nav_result_text}")

    def _dispatch(self) -> None:
        route = self._mission.prepared
        self._chunk = build_chunk(route, self._mission.target_index, self._mission.loop_iteration)
        if self._chunk is None:
            transition(self._mission, RoutePhase.COMPLETED)
            return
        self._target_offset = 0
        self._publish_paths()
        self._send_target()

    def _send_target(self) -> None:
        point = self._chunk.waypoints[self._target_offset]
        request = SetNavGoalLL.Request()
        request.lat, request.lon, request.yaw_deg = point.lat, point.lon, point.yaw_deg
        has_next = self._target_offset + 1 < len(self._chunk.waypoints)
        # The last waypoint of an intermediate chunk is contiguous with the
        # next chunk.  Only the mission's final waypoint requests Nav2's brake.
        after_chunk = next_start(self._mission.prepared, self._chunk)
        request.suppress_success_brake = has_next or self._mission.prepared.loop or after_chunk < len(self._mission.prepared.waypoints)
        self._goal_epoch += 1
        epoch = self._goal_epoch
        future = self._set_goal.call_async(request)
        future.add_done_callback(lambda done: self._on_goal_request(done, epoch))

    def _on_goal_request(self, future, epoch: int) -> None:
        with self._lock:
            if epoch != self._goal_epoch or self._mission.phase != RoutePhase.ACTIVE:
                return
            try:
                result = future.result()
                if result is None or not result.ok:
                    raise RuntimeError("empty response" if result is None else result.error)
            except Exception as exc:
                self._pause(f"NAV_GOAL_REJECTED: {exc}")

    def _advance(self) -> None:
        self._mission.reached += 1
        self._target_offset += 1
        if self._target_offset < len(self._chunk.waypoints):
            self._send_target()
            return
        self._mission.target_index = next_start(self._mission.prepared, self._chunk)
        self._mission.chunk_id += 1
        if self._mission.prepared.loop and self._mission.target_index == 0:
            self._mission.loop_iteration += 1
        self._dispatch()

    def _pause(self, reason: str) -> None:
        if self._mission.phase == RoutePhase.ACTIVE:
            transition(self._mission, RoutePhase.PAUSED, reason)
        self._goal_epoch += 1
        future = self._cancel_goal.call_async(CancelNavGoal.Request())
        future.add_done_callback(self._log_failed_cancel)

    def _cancel(self, _request, response):
        with self._lock:
            self._preparation_epoch += 1
            self._preparation = None
            if self._mission.phase == RoutePhase.ACTIVE:
                transition(self._mission, RoutePhase.CANCELLED, "cancelled")
            self._goal_epoch += 1
            self._cancel_goal.call_async(CancelNavGoal.Request()).add_done_callback(self._log_failed_cancel)
            self._brake.call_async(BrakeNav.Request(duration_s=0.25, brake_pct=100)).add_done_callback(self._log_failed_brake)
        response.ok, response.error = True, ""
        return response

    def _log_failed_cancel(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.ok:
                self.get_logger().error("route cancellation was rejected")
        except Exception as exc:
            self.get_logger().error(f"route cancellation failed: {exc}")

    def _log_failed_brake(self, future) -> None:
        try:
            result = future.result()
            if result is None or not result.ok:
                self.get_logger().error("route brake was rejected")
        except Exception as exc:
            self.get_logger().error(f"route brake failed: {exc}")

    def _publish_paths(self) -> None:
        for publisher, points in ((self._mission_path, self._mission.prepared.waypoints), (self._chunk_path, self._chunk.waypoints)):
            message = Path()
            message.header.frame_id = "map"
            message.header.stamp = self.get_clock().now().to_msg()
            for point in points:
                pose = PoseStamped()
                pose.header = message.header
                pose.pose.position.x, pose.pose.position.y = point.map_x, point.map_y
                pose.pose.orientation.w = 1.0
                message.poses.append(pose)
            publisher.publish(message)

    def _state(self, _request, response):
        with self._lock:
            mission, prepared = self._mission, self._mission.prepared
            response.ok, response.error = True, ""
            response.active = mission.phase == RoutePhase.ACTIVE
            response.paused = mission.phase == RoutePhase.PAUSED
            response.loop = bool(prepared and prepared.loop)
            response.status, response.mission_id = mission.phase.value, mission.mission_id
            response.chunk_id, response.loop_iteration = mission.chunk_id, mission.loop_iteration
            response.reached_checkpoint_count = mission.reached
            response.input_waypoint_count = 0 if prepared is None else prepared.input_count
            response.expanded_waypoint_count = 0 if prepared is None else len(prepared.waypoints)
            response.current_start_index = mission.target_index
            response.current_target_index = mission.target_index + self._target_offset
            response.active_chunk_size = 0 if self._chunk is None else len(self._chunk.waypoints)
            response.leg_spacing_m = 0.0 if prepared is None else prepared.leg_spacing_m
            response.chunk_span_m = 0.0 if prepared is None else prepared.chunk_span_m
            response.chunk_max_waypoints = 0 if prepared is None else prepared.chunk_max_waypoints
            response.blocked_reason_text = mission.pause_reason
            response.current_checkpoint_index = mission.progress.checkpoint_index
            response.current_progress_expanded_index = mission.progress.expanded_index
            response.current_progress_ratio = mission.progress.ratio
            response.cross_track_error_m = mission.progress.cross_track_error_m
            response.distance_to_target_m = mission.progress.distance_to_target_m
            if prepared is not None:
                response.mission_lats = [point.lat for point in prepared.waypoints]
                response.mission_lons = [point.lon for point in prepared.waypoints]
                response.mission_yaws_deg = [point.yaw_deg for point in prepared.waypoints]
                response.mission_action_jsons = [point.action_json for point in prepared.waypoints]
                response.mission_waypoint_roles = [point.role for point in prepared.waypoints]
                response.mission_key_flags = [point.key for point in prepared.waypoints]
                response.mission_input_indices = [point.input_index for point in prepared.waypoints]
            if self._chunk is not None:
                response.active_lats = [point.lat for point in self._chunk.waypoints]
                response.active_lons = [point.lon for point in self._chunk.waypoints]
                response.active_yaws_deg = [point.yaw_deg for point in self._chunk.waypoints]
        return response

    @staticmethod
    def _steady_now() -> float:
        from time import monotonic
        return monotonic()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteExecutorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
