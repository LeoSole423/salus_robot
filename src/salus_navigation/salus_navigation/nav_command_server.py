"""Safety-gated arbitration between autonomous and manual velocity commands."""

from __future__ import annotations

import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from salus_interfaces.msg import CmdVelFinal, NavEvent, NavTelemetry
from salus_interfaces.srv import BrakeNav, GetNavState, SetManualMode


def clamp_brake_pct(value: int) -> int:
    return max(0, min(100, int(value)))


def diagnostic_level(value: int | bytes) -> int:
    if isinstance(value, bytes):
        return int.from_bytes(value, byteorder="little", signed=False)
    return int(value)


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

    def scan_is_fresh(self, now_s: float) -> bool:
        return self.scan_stamp_s is not None and now_s - self.scan_stamp_s <= self.monitor_timeout_s

    def set_scan_received(self, now_s: float) -> None:
        self.scan_stamp_s = now_s

    def set_monitor_state(self, message: CollisionMonitorState, now_s: float) -> None:
        self.monitor_action = int(message.action_type)
        self.monitor_polygon = str(message.polygon_name)

    def set_manual_mode(self, enabled: bool) -> None:
        self.manual_enabled = bool(enabled)
        self.manual_timeout_stop_sent = False
        if not self.manual_enabled:
            self.manual_command = None
            self.manual_stamp_s = None

    def accept_manual(self, message: CmdVelFinal, now_s: float) -> CmdVelFinal | None:
        if not self.manual_enabled:
            return None
        command = CmdVelFinal()
        command.twist = message.twist
        command.brake_pct = clamp_brake_pct(message.brake_pct)
        command.source = CmdVelFinal.SOURCE_MANUAL
        self.manual_command = command
        self.manual_stamp_s = now_s
        self.manual_timeout_stop_sent = False
        return command

    def automatic_output(self, message: Twist, now_s: float) -> tuple[CmdVelFinal | None, str]:
        if self.manual_enabled:
            return None, "manual_enabled"
        if not self.scan_is_fresh(now_s):
            return self.stop(CmdVelFinal.SOURCE_SAFETY), "scan_stale"
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
        command = CmdVelFinal()
        command.twist.linear.x = float(linear_x)
        command.twist.angular.z = float(angular_z)
        command.brake_pct = clamp_brake_pct(brake_pct)
        command.source = int(source)
        return command

    @classmethod
    def stop(cls, source: int, brake_pct: int = 0) -> CmdVelFinal:
        return cls.command(0.0, 0.0, brake_pct, source)


class NavCommandServer(Node):
    """ROS boundary for the command-arbitration migration slice."""

    def __init__(self) -> None:
        super().__init__("nav_command_server")
        self.declare_parameter("cmd_vel_safe_topic", "/cmd_vel_safe")
        self.declare_parameter("teleop_cmd_topic", "/cmd_vel_teleop")
        self.declare_parameter("cmd_vel_final_topic", "/cmd_vel_final")
        self.declare_parameter("collision_monitor_state_topic", "/collision_monitor_state")
        self.declare_parameter("safety_scan_topic", "/scan_clean")
        self.declare_parameter("manual_cmd_timeout_s", 0.4)
        self.declare_parameter("collision_monitor_timeout_s", 1.0)
        self.declare_parameter("manual_watchdog_hz", 10.0)
        self.declare_parameter("nav_telemetry_hz", 5.0)
        self.declare_parameter("brake_hold_publish_hz", 10.0)
        self.declare_parameter("telemetry_topic", "/nav_command_server/telemetry")
        self.declare_parameter("event_topic", "/nav_command_server/events")
        self.declare_parameter("brake_service", "/nav_command_server/brake")
        self.declare_parameter("set_manual_mode_service", "/nav_command_server/set_manual_mode")
        self.declare_parameter("get_state_service", "/nav_command_server/get_state")

        p = lambda name: self.get_parameter(name).value
        self._lock = threading.Lock()
        self._arbiter = CommandArbiter(
            manual_timeout_s=float(p("manual_cmd_timeout_s")),
            monitor_timeout_s=float(p("collision_monitor_timeout_s")),
        )
        self._last_safe: Twist | None = None
        self._last_safe_stamp_s: float | None = None
        self._event_id = 0
        self._failure_code = ""
        self._failure_component = ""
        self._brake_cancel: threading.Event | None = None

        self._final_pub = self.create_publisher(CmdVelFinal, str(p("cmd_vel_final_topic")), 10)
        self._telemetry_pub = self.create_publisher(NavTelemetry, str(p("telemetry_topic")), 10)
        self._event_pub = self.create_publisher(NavEvent, str(p("event_topic")), 10)
        self.create_subscription(Twist, str(p("cmd_vel_safe_topic")), self._on_safe, 10)
        self.create_subscription(CmdVelFinal, str(p("teleop_cmd_topic")), self._on_teleop, 10)
        self.create_subscription(CollisionMonitorState, str(p("collision_monitor_state_topic")), self._on_monitor_state, 10)
        self.create_subscription(LaserScan, str(p("safety_scan_topic")), self._on_scan, qos_profile_sensor_data)
        self.create_service(BrakeNav, str(p("brake_service")), self._on_brake)
        self.create_service(SetManualMode, str(p("set_manual_mode_service")), self._on_set_manual_mode)
        self.create_service(GetNavState, str(p("get_state_service")), self._on_get_state)
        self.create_timer(1.0 / max(1.0, float(p("manual_watchdog_hz"))), self._manual_watchdog)
        self.create_timer(1.0 / max(1.0, float(p("nav_telemetry_hz"))), self._publish_telemetry)
        self._brake_hold_hz = max(1.0, float(p("brake_hold_publish_hz")))
        self.get_logger().info("nav command arbitration ready: /cmd_vel_safe and /cmd_vel_teleop -> /cmd_vel_final")

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
        event.code = code
        event.message = message
        event.event_id = self._event_id
        event.details = [KeyValue(key=str(key), value=str(value)) for key, value in details.items()]
        self._event_pub.publish(event)

    def _on_monitor_state(self, message: CollisionMonitorState) -> None:
        with self._lock:
            self._arbiter.set_monitor_state(message, self._now_s())

    def _on_scan(self, _message: LaserScan) -> None:
        with self._lock:
            self._arbiter.set_scan_received(self._now_s())

    def _on_safe(self, message: Twist) -> None:
        with self._lock:
            now_s = self._now_s()
            self._last_safe = message
            self._last_safe_stamp_s = now_s
            command, reason = self._arbiter.automatic_output(message, now_s)
            if reason == "scan_stale":
                self._failure_code = "SAFETY_SCAN_STALE"
                self._failure_component = "perception"
            elif reason == "collision_stop":
                self._failure_code = "COLLISION_STOP"
                self._failure_component = "collision_monitor"
            else:
                self._failure_code = ""
                self._failure_component = ""
        if command is not None:
            self._publish(command)
        if reason == "scan_stale":
            self._event(DiagnosticStatus.ERROR, "SAFETY_SCAN_STALE", "automatic command stopped because /scan_clean is stale")

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
            self._event(DiagnosticStatus.WARN, "MANUAL_COMMAND_TIMEOUT", "manual command watchdog stopped the robot")

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
            with self._lock:
                if self._brake_cancel is cancel:
                    self._brake_cancel = None

        threading.Thread(target=publish_hold, daemon=True, name="nav_brake_hold").start()

    def _on_brake(self, request: BrakeNav.Request, response: BrakeNav.Response) -> BrakeNav.Response:
        duration_s = max(0.0, float(request.duration_s))
        brake_pct = clamp_brake_pct(request.brake_pct)
        self._start_brake_hold(duration_s, brake_pct)
        response.ok = True
        response.error = ""
        self._event(DiagnosticStatus.WARN, "BRAKE_REQUESTED", "brake hold requested", duration_s=duration_s, brake_pct=brake_pct)
        return response

    def _on_set_manual_mode(self, request: SetManualMode.Request, response: SetManualMode.Response) -> SetManualMode.Response:
        with self._lock:
            if self._brake_cancel is not None:
                self._brake_cancel.set()
                self._brake_cancel = None
            self._arbiter.set_manual_mode(request.enabled)
            response.enabled_after = self._arbiter.manual_enabled
        response.ok = True
        response.error = ""
        self._event(DiagnosticStatus.OK, "MANUAL_MODE_CHANGED", "manual mode changed", enabled=response.enabled_after)
        return response

    def _on_get_state(self, _request: GetNavState.Request, response: GetNavState.Response) -> GetNavState.Response:
        with self._lock:
            manual = self._arbiter.manual_command
            response.ok = True
            response.error = ""
            response.goal_active = False
            response.manual_enabled = self._arbiter.manual_enabled
            response.manual_linear_x_cmd = 0.0 if manual is None else manual.twist.linear.x
            response.manual_angular_z_cmd = 0.0 if manual is None else manual.twist.angular.z
            response.cmd_vel_available = self._last_safe is not None
            response.cmd_vel_linear_x = 0.0 if self._last_safe is None else self._last_safe.linear.x
            response.cmd_vel_angular_z = 0.0 if self._last_safe is None else self._last_safe.angular.z
            response.robot_lat = 0.0
            response.robot_lon = 0.0
        return response

    def _publish_telemetry(self) -> None:
        with self._lock:
            now_s = self._now_s()
            message = NavTelemetry()
            message.goal_active = False
            message.manual_enabled = self._arbiter.manual_enabled
            message.auto_mode = "manual" if self._arbiter.manual_enabled else "safety_arbitration"
            message.active_action = "none"
            manual = self._arbiter.manual_command
            message.manual_linear_x_cmd = 0.0 if manual is None else manual.twist.linear.x
            message.manual_angular_z_cmd = 0.0 if manual is None else manual.twist.angular.z
            message.cmd_vel_available = self._last_safe is not None
            message.cmd_vel_safe_fresh = self._last_safe_stamp_s is not None and now_s - self._last_safe_stamp_s <= 1.0
            message.cmd_vel_safe_age_s = float("inf") if self._last_safe_stamp_s is None else now_s - self._last_safe_stamp_s
            message.cmd_vel_linear_x = 0.0 if self._last_safe is None else self._last_safe.linear.x
            message.cmd_vel_angular_z = 0.0 if self._last_safe is None else self._last_safe.angular.z
            message.collision_stop_active = self._arbiter.monitor_action == CollisionMonitorState.STOP
            message.robot_pose_available = False
            message.gps_fix_available = False
            message.gps_age_s = float("inf")
            message.robot_lat = 0.0
            message.robot_lon = 0.0
            message.nav_result_status = 0
            message.nav_result_text = "navigation core not migrated"
            message.nav_result_event_id = self._event_id
            message.failure_code = self._failure_code
            message.failure_component = self._failure_component
        self._telemetry_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavCommandServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
