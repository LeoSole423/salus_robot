"""Safety arbitration and single-goal Nav2 orchestration for SALUS."""

from __future__ import annotations

import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from robot_localization.srv import FromLL
from sensor_msgs.msg import LaserScan, NavSatFix
from salus_interfaces.msg import CmdVelFinal, NavEvent, NavTelemetry, PathHealth
from salus_interfaces.srv import (
    BrakeNav,
    CancelNavGoal,
    GetNavState,
    SetManualMode,
    SetNavGoalLL,
)


def clamp_brake_pct(value: int) -> int:
    return max(0, min(100, int(value)))


def diagnostic_level(value: int | bytes) -> int:
    return int.from_bytes(value, byteorder="little", signed=False) if isinstance(value, bytes) else int(value)


class CommandArbiter:
    """Pure command policy, isolated from ROS publishers and clocks."""

    def __init__(self, *, manual_timeout_s: float, monitor_timeout_s: float) -> None:
        self.manual_timeout_s = max(0.1, float(manual_timeout_s))
        self.monitor_timeout_s = max(0.1, float(monitor_timeout_s))
        self.manual_enabled = False
        self.manual_command: CmdVelFinal | None = None
        self.manual_stamp_s: float | None = None
        self.manual_timeout_stop_sent = False
        self.scan_stamp_s: float | None = None
        self.monitor_action = CollisionMonitorState.DO_NOTHING
        self.monitor_polygon = ""
        self.path_health_state = PathHealth.KEEP_PATH
        self.path_health_reason = "path_health_unavailable"

    def scan_is_fresh(self, now_s: float) -> bool:
        return self.scan_stamp_s is not None and now_s - self.scan_stamp_s <= self.monitor_timeout_s

    def set_scan_received(self, now_s: float) -> None:
        self.scan_stamp_s = now_s

    def set_monitor_state(self, message: CollisionMonitorState, _now_s: float | None = None) -> None:
        self.monitor_action = int(message.action_type)
        self.monitor_polygon = str(message.polygon_name)

    def set_path_health(self, message: PathHealth) -> None:
        self.path_health_state = int(message.state)
        self.path_health_reason = str(message.reason)

    def set_manual_mode(self, enabled: bool) -> None:
        self.manual_enabled = bool(enabled)
        self.manual_timeout_stop_sent = False
        if not self.manual_enabled:
            self.manual_command = None
            self.manual_stamp_s = None

    def accept_manual(self, message: CmdVelFinal, now_s: float) -> CmdVelFinal | None:
        if not self.manual_enabled:
            return None
        self.manual_command = self.command(message.twist.linear.x, message.twist.angular.z, message.brake_pct, CmdVelFinal.SOURCE_MANUAL)
        self.manual_stamp_s = now_s
        self.manual_timeout_stop_sent = False
        return self.manual_command

    def automatic_output(self, message: Twist, now_s: float) -> tuple[CmdVelFinal | None, str]:
        if self.manual_enabled:
            return None, "manual_enabled"
        if not self.scan_is_fresh(now_s):
            return self.stop(CmdVelFinal.SOURCE_SAFETY), "scan_stale"
        if self.path_health_state == PathHealth.STOP_AND_WAIT:
            return self.stop(CmdVelFinal.SOURCE_SAFETY), "path_health_stop"
        if self.monitor_action == CollisionMonitorState.STOP:
            return self.stop(CmdVelFinal.SOURCE_SAFETY), "collision_stop"
        return self.command(message.linear.x, message.angular.z, 0, CmdVelFinal.SOURCE_AUTO), "auto"

    def manual_watchdog_output(self, now_s: float) -> CmdVelFinal | None:
        if not self.manual_enabled or self.manual_stamp_s is None:
            return None
        if now_s - self.manual_stamp_s <= self.manual_timeout_s or self.manual_timeout_stop_sent:
            return None
        self.manual_timeout_stop_sent = True
        return self.stop(CmdVelFinal.SOURCE_MANUAL)

    @staticmethod
    def command(linear_x: float, angular_z: float, brake_pct: int, source: int) -> CmdVelFinal:
        message = CmdVelFinal()
        message.twist.linear.x = float(linear_x)
        message.twist.angular.z = float(angular_z)
        message.brake_pct = clamp_brake_pct(brake_pct)
        message.source = int(source)
        return message

    @classmethod
    def stop(cls, source: int, brake_pct: int = 0) -> CmdVelFinal:
        return cls.command(0.0, 0.0, brake_pct, source)


class NavCommandServer(Node):
    """ROS boundary for safety arbitration and the migrated Nav2 goal API."""

    def __init__(self) -> None:
        super().__init__("nav_command_server")
        for name, value in {
            "cmd_vel_safe_topic": "/cmd_vel_safe", "teleop_cmd_topic": "/cmd_vel_teleop",
            "cmd_vel_final_topic": "/cmd_vel_final", "collision_monitor_state_topic": "/collision_monitor_state",
            "safety_scan_topic": "/scan_clean", "gps_topic": "/gps/fix",
            "path_health_topic": "/path_health",
            "manual_cmd_timeout_s": 0.4, "collision_monitor_timeout_s": 1.0,
            "manual_watchdog_hz": 10.0, "nav_telemetry_hz": 5.0, "brake_hold_publish_hz": 10.0,
            "telemetry_topic": "/nav_command_server/telemetry", "event_topic": "/nav_command_server/events",
            "set_goal_service": "/nav_command_server/set_goal_ll", "cancel_goal_service": "/nav_command_server/cancel_goal",
            "brake_service": "/nav_command_server/brake", "set_manual_mode_service": "/nav_command_server/set_manual_mode",
            "get_state_service": "/nav_command_server/get_state", "fromll_service": "/fromLL",
            "fromll_service_fallback": "/navsat_transform/fromLL", "fromll_timeout_s": 2.0,
            "navigate_action": "/navigate_to_pose",
        }.items():
            self.declare_parameter(name, value)
        p = lambda name: self.get_parameter(name).value
        self._lock = threading.Lock()
        self._arbiter = CommandArbiter(manual_timeout_s=float(p("manual_cmd_timeout_s")), monitor_timeout_s=float(p("collision_monitor_timeout_s")))
        self._last_safe: Twist | None = None
        self._last_safe_stamp_s: float | None = None
        self._last_fix: NavSatFix | None = None
        self._keepout_mask: OccupancyGrid | None = None
        self._last_fix_stamp_s: float | None = None
        self._event_id = 0
        self._failure_code = ""
        self._failure_component = ""
        self._brake_cancel: threading.Event | None = None
        self._goal_epoch = 0
        self._goal_pending = False
        self._goal_handle = None
        self._goal_result_status = GoalStatus.STATUS_UNKNOWN
        self._goal_result_text = "idle"
        self._suppress_success_brake = False

        self._final_pub = self.create_publisher(CmdVelFinal, str(p("cmd_vel_final_topic")), 10)
        self._telemetry_pub = self.create_publisher(NavTelemetry, str(p("telemetry_topic")), 10)
        self._event_pub = self.create_publisher(NavEvent, str(p("event_topic")), 10)
        self.create_subscription(Twist, str(p("cmd_vel_safe_topic")), self._on_safe, 10)
        self.create_subscription(CmdVelFinal, str(p("teleop_cmd_topic")), self._on_teleop, 10)
        self.create_subscription(CollisionMonitorState, str(p("collision_monitor_state_topic")), self._on_monitor_state, 10)
        self.create_subscription(LaserScan, str(p("safety_scan_topic")), self._on_scan, qos_profile_sensor_data)
        self.create_subscription(NavSatFix, str(p("gps_topic")), self._on_gps, qos_profile_sensor_data)
        self.create_subscription(PathHealth, str(p("path_health_topic")), self._on_path_health, 10)
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()
        keepout_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, "/keepout_filter_mask", self._on_keepout_mask, keepout_qos)
        self.create_service(BrakeNav, str(p("brake_service")), self._on_brake, callback_group=self._service_group)
        self.create_service(SetManualMode, str(p("set_manual_mode_service")), self._on_set_manual_mode, callback_group=self._service_group)
        self.create_service(GetNavState, str(p("get_state_service")), self._on_get_state, callback_group=self._service_group)
        self.create_service(SetNavGoalLL, str(p("set_goal_service")), self._on_set_goal, callback_group=self._service_group)
        self.create_service(CancelNavGoal, str(p("cancel_goal_service")), self._on_cancel_goal, callback_group=self._service_group)
        self._fromll_clients = [
            self.create_client(FromLL, str(p("fromll_service")), callback_group=self._client_group),
            self.create_client(FromLL, str(p("fromll_service_fallback")), callback_group=self._client_group),
        ]
        self._fromll_timeout_s = max(0.1, float(p("fromll_timeout_s")))
        self._navigate_client = ActionClient(self, NavigateToPose, str(p("navigate_action")), callback_group=self._client_group)
        self.create_timer(1.0 / max(1.0, float(p("manual_watchdog_hz"))), self._manual_watchdog)
        self.create_timer(1.0 / max(1.0, float(p("nav_telemetry_hz"))), self._publish_telemetry)
        self._brake_hold_hz = max(1.0, float(p("brake_hold_publish_hz")))
        self.get_logger().info("nav command server ready: safe/manual commands and LL single goals")

    def _now_s(self) -> float:
        return time.monotonic()

    def _publish(self, command: CmdVelFinal) -> None:
        self._final_pub.publish(command)

    def _event(self, severity: int, code: str, message: str, **details: str) -> None:
        self._event_id += 1
        event = NavEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.severity = diagnostic_level(severity)
        event.component = "nav_command_server"
        event.code, event.message, event.event_id = code, message, self._event_id
        event.details = [KeyValue(key=str(key), value=str(value)) for key, value in details.items()]
        self._event_pub.publish(event)

    def _on_monitor_state(self, message: CollisionMonitorState) -> None:
        with self._lock:
            self._arbiter.set_monitor_state(message)

    def _on_scan(self, _message: LaserScan) -> None:
        with self._lock:
            self._arbiter.set_scan_received(self._now_s())

    def _on_gps(self, message: NavSatFix) -> None:
        if math.isfinite(message.latitude) and math.isfinite(message.longitude):
            with self._lock:
                self._last_fix, self._last_fix_stamp_s = message, self._now_s()

    def _on_path_health(self, message: PathHealth) -> None:
        with self._lock:
            self._arbiter.set_path_health(message)

    def _on_keepout_mask(self, message: OccupancyGrid) -> None:
        with self._lock:
            self._keepout_mask = message

    def _goal_in_keepout(self, x_m: float, y_m: float) -> bool:
        with self._lock:
            mask = self._keepout_mask
        if mask is None or mask.info.resolution <= 0.0:
            return False
        col = math.floor((x_m - mask.info.origin.position.x) / mask.info.resolution)
        row = math.floor((y_m - mask.info.origin.position.y) / mask.info.resolution)
        if not (0 <= col < mask.info.width and 0 <= row < mask.info.height):
            return False
        return mask.data[row * mask.info.width + col] >= 100

    def _on_safe(self, message: Twist) -> None:
        with self._lock:
            now_s = self._now_s()
            self._last_safe, self._last_safe_stamp_s = message, now_s
            command, reason = self._arbiter.automatic_output(message, now_s)
            self._failure_code, self._failure_component = (("SAFETY_SCAN_STALE", "perception") if reason == "scan_stale" else ("COLLISION_STOP", "collision_monitor") if reason == "collision_stop" else ("", ""))
        if command is not None:
            self._publish(command)
        if reason == "scan_stale":
            self._event(DiagnosticStatus.ERROR, "SAFETY_SCAN_STALE", "automatic command stopped because /scan_clean is stale")
        elif reason == "path_health_stop":
            self._event(DiagnosticStatus.ERROR, "PATH_HEALTH_STOP", "automatic command stopped while global path data is unavailable")

    def _on_teleop(self, message: CmdVelFinal) -> None:
        with self._lock:
            command = self._arbiter.accept_manual(message, self._now_s())
        if command is not None:
            self._publish(command)

    def _manual_watchdog(self) -> None:
        with self._lock:
            command = self._arbiter.manual_watchdog_output(self._now_s())
        if command is not None:
            self._publish(command)
            self._event(DiagnosticStatus.WARN, "MANUAL_WATCHDOG_STOP", "manual command watchdog stopped the robot")

    def _start_brake_hold(self, duration_s: float, brake_pct: int) -> None:
        cancel = threading.Event()
        with self._lock:
            if self._brake_cancel is not None:
                self._brake_cancel.set()
            self._brake_cancel = cancel
        def publish_hold() -> None:
            deadline = time.monotonic() + max(0.0, duration_s)
            while not cancel.is_set() and time.monotonic() < deadline:
                self._publish(CommandArbiter.stop(CmdVelFinal.SOURCE_SAFETY, brake_pct))
                cancel.wait(1.0 / self._brake_hold_hz)
        threading.Thread(target=publish_hold, daemon=True, name="nav_brake_hold").start()

    def _cancel_goal(self, reason: str, *, apply_brake: bool) -> bool:
        with self._lock:
            handle, was_active = self._goal_handle, self._goal_pending or self._goal_handle is not None
            self._goal_epoch += 1
            self._goal_pending, self._goal_handle = False, None
            self._goal_result_text = reason
        if handle is not None:
            handle.cancel_goal_async()
        if apply_brake:
            self._publish(CommandArbiter.stop(CmdVelFinal.SOURCE_SAFETY, 100))
            self._start_brake_hold(0.4, 100)
        return was_active

    def _on_brake(self, request: BrakeNav.Request, response: BrakeNav.Response) -> BrakeNav.Response:
        self._start_brake_hold(max(0.0, request.duration_s), clamp_brake_pct(request.brake_pct))
        response.ok, response.error = True, ""
        self._event(DiagnosticStatus.WARN, "BRAKE_REQUESTED", "brake hold requested", duration_s=request.duration_s, brake_pct=request.brake_pct)
        return response

    def _on_set_manual_mode(self, request: SetManualMode.Request, response: SetManualMode.Response) -> SetManualMode.Response:
        if request.enabled:
            cancelled = self._cancel_goal("manual takeover", apply_brake=True)
            if cancelled:
                self._event(DiagnosticStatus.WARN, "GOAL_CANCELLED", "goal cancelled by manual takeover")
        with self._lock:
            if self._brake_cancel is not None:
                self._brake_cancel.set()
                self._brake_cancel = None
            self._arbiter.set_manual_mode(request.enabled)
            response.enabled_after = self._arbiter.manual_enabled
        response.ok, response.error = True, ""
        self._event(DiagnosticStatus.OK, "MANUAL_MODE_CHANGED", "manual mode changed", enabled=response.enabled_after)
        return response

    @staticmethod
    def _single_waypoint(request: SetNavGoalLL.Request) -> tuple[tuple[float, float, float] | None, str]:
        arrays = (list(request.lats), list(request.lons), list(request.yaws_deg))
        populated = any(values for values in arrays)
        if request.loop:
            return None, "loop goals belong to the future missions subsystem"
        if populated:
            if len(arrays[0]) != len(arrays[1]) or len(arrays[0]) != len(arrays[2]):
                return None, "waypoint arrays must have equal lengths"
            if len(arrays[0]) != 1:
                return None, "multiple waypoints belong to the future missions subsystem"
            waypoint = arrays[0][0], arrays[1][0], arrays[2][0]
        else:
            waypoint = request.lat, request.lon, request.yaw_deg
        return (tuple(float(value) for value in waypoint), "") if all(math.isfinite(value) for value in waypoint) else (None, "invalid waypoint values")

    def _on_set_goal(self, request: SetNavGoalLL.Request, response: SetNavGoalLL.Response) -> SetNavGoalLL.Response:
        waypoint, error = self._single_waypoint(request)
        with self._lock:
            manual = self._arbiter.manual_enabled
        if manual:
            response.ok, response.error = False, "manual control enabled; disable manual mode to send goals"
            return response
        if waypoint is None:
            response.ok, response.error = False, error
            self._event(DiagnosticStatus.WARN, "GOAL_REJECTED", error)
            return response
        client = next((candidate for candidate in self._fromll_clients if candidate.service_is_ready()), None)
        if client is None:
            response.ok, response.error = False, "fromLL service unavailable"
            self._event(DiagnosticStatus.ERROR, "FROMLL_FAILED", response.error)
            return response
        if not self._navigate_client.server_is_ready():
            response.ok, response.error = False, "NavigateToPose action server unavailable"
            self._event(DiagnosticStatus.ERROR, "ACTION_SERVER_UNAVAILABLE", response.error)
            return response
        point, error = self._convert_goal_to_map(client, waypoint[0], waypoint[1])
        if point is None:
            response.ok, response.error = False, error
            self._event(DiagnosticStatus.ERROR, "FROMLL_FAILED", error)
            return response
        if self._goal_in_keepout(point.x, point.y):
            response.ok, response.error = False, "goal lies in keepout zone"
            self._event(DiagnosticStatus.WARN, "GOAL_REJECTED", response.error)
            return response
        self._cancel_goal("replaced by new goal", apply_brake=False)
        with self._lock:
            self._goal_epoch += 1
            epoch = self._goal_epoch
            self._goal_pending = True
            self._suppress_success_brake = bool(request.suppress_success_brake)
            self._goal_result_status, self._goal_result_text = GoalStatus.STATUS_UNKNOWN, "sending navigation goal"
        self._send_map_goal(point, waypoint[2], epoch)
        response.ok, response.error = True, ""
        self._event(DiagnosticStatus.OK, "GOAL_REQUESTED", "geographic goal requested", lat=waypoint[0], lon=waypoint[1])
        return response

    def _convert_goal_to_map(self, primary_client, latitude: float, longitude: float):
        """Convert before acknowledging the service, including the legacy fallback."""
        request = FromLL.Request()
        request.ll_point.latitude, request.ll_point.longitude, request.ll_point.altitude = latitude, longitude, 0.0
        candidates = [primary_client] + [item for item in self._fromll_clients if item is not primary_client]
        last_error = "fromLL service unavailable"
        for client in candidates:
            if not client.wait_for_service(timeout_sec=self._fromll_timeout_s):
                continue
            completed = threading.Event()
            future = client.call_async(request)
            future.add_done_callback(lambda _future: completed.set())
            if not completed.wait(self._fromll_timeout_s):
                last_error = "fromLL conversion timed out"
                continue
            try:
                return future.result().map_point, ""
            except Exception as exc:
                last_error = f"fromLL conversion failed: {exc}"
        return None, last_error

    @staticmethod
    def _quaternion_from_yaw(yaw_deg: float) -> tuple[float, float]:
        half = math.radians(yaw_deg) * 0.5
        return math.sin(half), math.cos(half)

    def _send_map_goal(self, point, yaw_deg: float, epoch: int) -> None:
        with self._lock:
            if epoch != self._goal_epoch or self._arbiter.manual_enabled:
                return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = point.x, point.y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = self._quaternion_from_yaw(yaw_deg)
        self._navigate_client.send_goal_async(goal).add_done_callback(lambda done: self._on_goal_response(done, epoch))

    def _on_goal_response(self, future, epoch: int) -> None:
        try:
            handle = future.result()
        except Exception as exc:
            handle = None
            self.get_logger().error(f"NavigateToPose request failed: {exc}")
        with self._lock:
            stale = epoch != self._goal_epoch or self._arbiter.manual_enabled
            if handle is None or not handle.accepted:
                if epoch == self._goal_epoch:
                    self._goal_pending, self._goal_result_text = False, "goal rejected"
                accepted = False
            else:
                accepted = True
                if not stale:
                    self._goal_pending, self._goal_handle, self._goal_result_text = False, handle, "navigating"
        if not accepted:
            self._event(DiagnosticStatus.ERROR, "GOAL_REJECTED", "NavigateToPose goal rejected")
            return
        if stale:
            handle.cancel_goal_async()
            return
        self._event(DiagnosticStatus.OK, "GOAL_ACCEPTED", "NavigateToPose goal accepted")
        handle.get_result_async().add_done_callback(lambda done: self._on_goal_result(done, epoch))

    def _on_goal_result(self, future, epoch: int) -> None:
        try:
            result = future.result()
            status = int(result.status)
        except Exception:
            status = GoalStatus.STATUS_ABORTED
        with self._lock:
            if epoch != self._goal_epoch:
                return
            self._goal_handle, self._goal_pending, self._goal_result_status = None, False, status
            self._goal_result_text = "succeeded" if status == GoalStatus.STATUS_SUCCEEDED else "cancelled" if status == GoalStatus.STATUS_CANCELED else "aborted"
            suppress_brake = self._suppress_success_brake
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._event(DiagnosticStatus.OK, "GOAL_RESULT_SUCCEEDED", "navigation goal succeeded")
            if not suppress_brake:
                self._start_brake_hold(0.25, 100)
        elif status == GoalStatus.STATUS_CANCELED:
            self._event(DiagnosticStatus.WARN, "GOAL_CANCELLED", "navigation goal cancelled")
        else:
            self._event(DiagnosticStatus.ERROR, "GOAL_RESULT_ABORTED", "navigation goal aborted")

    def _on_cancel_goal(self, _request: CancelNavGoal.Request, response: CancelNavGoal.Response) -> CancelNavGoal.Response:
        response.ok, response.error = True, ""
        if self._cancel_goal("cancelled by service", apply_brake=True):
            self._event(DiagnosticStatus.WARN, "GOAL_CANCELLED", "goal cancelled by service")
        return response

    def _on_get_state(self, _request: GetNavState.Request, response: GetNavState.Response) -> GetNavState.Response:
        with self._lock:
            manual, fix = self._arbiter.manual_command, self._last_fix
            response.ok, response.error = True, ""
            response.goal_active = self._goal_pending or self._goal_handle is not None
            response.manual_enabled = self._arbiter.manual_enabled
            response.manual_linear_x_cmd = 0.0 if manual is None else manual.twist.linear.x
            response.manual_angular_z_cmd = 0.0 if manual is None else manual.twist.angular.z
            response.cmd_vel_available = self._last_safe is not None
            response.cmd_vel_linear_x = 0.0 if self._last_safe is None else self._last_safe.linear.x
            response.cmd_vel_angular_z = 0.0 if self._last_safe is None else self._last_safe.angular.z
            response.robot_lat = 0.0 if fix is None else fix.latitude
            response.robot_lon = 0.0 if fix is None else fix.longitude
        return response

    def _publish_telemetry(self) -> None:
        with self._lock:
            now_s, manual, fix = self._now_s(), self._arbiter.manual_command, self._last_fix
            message = NavTelemetry()
            message.goal_active = self._goal_pending or self._goal_handle is not None
            message.manual_enabled = self._arbiter.manual_enabled
            message.auto_mode = "manual" if self._arbiter.manual_enabled else "navigate_to_pose" if message.goal_active else "safety_arbitration"
            message.active_action = "NavigateToPose" if message.goal_active else "none"
            message.manual_linear_x_cmd = 0.0 if manual is None else manual.twist.linear.x
            message.manual_angular_z_cmd = 0.0 if manual is None else manual.twist.angular.z
            message.cmd_vel_available = self._last_safe is not None
            message.cmd_vel_safe_fresh = self._last_safe_stamp_s is not None and now_s - self._last_safe_stamp_s <= 1.0
            message.cmd_vel_safe_age_s = float("inf") if self._last_safe_stamp_s is None else now_s - self._last_safe_stamp_s
            message.cmd_vel_linear_x = 0.0 if self._last_safe is None else self._last_safe.linear.x
            message.cmd_vel_angular_z = 0.0 if self._last_safe is None else self._last_safe.angular.z
            message.collision_stop_active = self._arbiter.monitor_action == CollisionMonitorState.STOP
            message.robot_pose_available = fix is not None
            message.gps_fix_available = fix is not None
            message.gps_age_s = float("inf") if self._last_fix_stamp_s is None else now_s - self._last_fix_stamp_s
            message.robot_lat = 0.0 if fix is None else fix.latitude
            message.robot_lon = 0.0 if fix is None else fix.longitude
            message.nav_result_status, message.nav_result_text = self._goal_result_status, self._goal_result_text
            message.nav_result_event_id, message.failure_code, message.failure_component = self._event_id, self._failure_code, self._failure_component
        self._telemetry_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavCommandServer()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
