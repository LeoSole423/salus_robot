"""Publish compact, API-level diagnostics for the active Nav2 stack."""

from __future__ import annotations

from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from lifecycle_msgs.msg import TransitionEvent
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Path
from rclpy.node import Node
from salus_interfaces.msg import NavEvent, NavTelemetry, PathHealth
from salus_navigation.nav_command_server import diagnostic_level


def plan_signature(path: Path) -> tuple[object, ...]:
    """Return a stable summary; timestamps alone must not create replans."""
    poses = path.poses
    if not poses:
        return (path.header.frame_id, 0)
    indices = sorted({0, len(poses) // 2, len(poses) - 1})
    samples = tuple(
        (round(poses[index].pose.position.x, 2), round(poses[index].pose.position.y, 2))
        for index in indices
    )
    return (path.header.frame_id, len(poses), samples)


@dataclass
class PlanReplanTracker:
    """Identify material plan changes while a NavigateToPose goal is active."""

    active_signature: tuple[object, ...] | None = None

    def observe(self, path: Path, *, goal_active: bool) -> bool:
        if not goal_active:
            self.active_signature = None
            return False
        signature = plan_signature(path)
        if self.active_signature is None:
            self.active_signature = signature
            return False
        changed = signature != self.active_signature
        self.active_signature = signature
        return changed


class NavObserver(Node):
    """Keep diagnostics outside of Nav2 lifecycle and command ownership."""

    def __init__(self) -> None:
        super().__init__("nav_observer")
        self.declare_parameter("event_topic", "/nav_command_server/events")
        self._event_id = 0
        self._goal_active = False
        self._collision_stopped = False
        self._tracker = PlanReplanTracker()
        self._events = self.create_publisher(NavEvent, str(self.get_parameter("event_topic").value), 10)
        self.create_subscription(NavTelemetry, "/nav_command_server/telemetry", self._on_telemetry, 10)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._on_collision, 10)
        self.create_subscription(PathHealth, "/path_health", self._on_path_health, 10)
        for name in ("planner_server", "controller_server", "bt_navigator", "behavior_server"):
            self.create_subscription(TransitionEvent, f"/{name}/transition_event", lambda message, node_name=name: self._on_transition(node_name, message), 10)

    def _emit(self, severity: int, code: str, message: str, **details: object) -> None:
        self._event_id += 1
        event = NavEvent()
        event.stamp = self.get_clock().now().to_msg()
        event.severity, event.component = diagnostic_level(severity), "nav_observer"
        event.code, event.message, event.event_id = code, message, self._event_id
        event.details = [KeyValue(key=str(key), value=str(value)) for key, value in details.items()]
        self._events.publish(event)

    def _on_telemetry(self, message: NavTelemetry) -> None:
        self._goal_active = bool(message.goal_active)

    def _on_plan(self, path: Path) -> None:
        if self._tracker.observe(path, goal_active=self._goal_active):
            self._emit(DiagnosticStatus.OK, "REPLAN_OBSERVED", "Nav2 published a materially different plan", poses=len(path.poses), frame=path.header.frame_id)

    def _on_collision(self, message: CollisionMonitorState) -> None:
        stopped = int(message.action_type) == CollisionMonitorState.STOP
        if stopped != self._collision_stopped:
            self._collision_stopped = stopped
            self._emit(
                DiagnosticStatus.WARN if stopped else DiagnosticStatus.OK,
                "NAV_BLOCKED" if stopped else "NAV_UNBLOCKED",
                "collision monitor stopped autonomous motion" if stopped else "collision monitor cleared autonomous motion",
                polygon=message.polygon_name,
            )

    def _on_path_health(self, message: PathHealth) -> None:
        details = {
            "reason": message.reason,
            "costmap_age_s": message.costmap_age_s,
            "max_cost": message.max_cost,
            "checked_samples": message.checked_samples,
            "cross_track_error_m": message.cross_track_error_m,
        }
        if message.reason.startswith("replan_accepted:"):
            self._emit(DiagnosticStatus.OK, "REPLAN_ACCEPTED", "candidate path accepted", **details)
        elif message.reason.startswith("replan_rejected:"):
            self._emit(DiagnosticStatus.WARN, "REPLAN_REJECTED", "candidate path rejected", **details)
        elif message.state == PathHealth.REPLAN:
            self._emit(DiagnosticStatus.WARN, "REPLAN_REQUESTED", "active path health requires replanning", **details)
        elif message.state == PathHealth.STOP_AND_WAIT:
            self._emit(DiagnosticStatus.ERROR, "PATH_STOP_AND_WAIT", "automatic navigation is waiting for valid path data", **details)

    def _on_transition(self, node_name: str, message: TransitionEvent) -> None:
        self._emit(
            DiagnosticStatus.OK,
            "LIFECYCLE_TRANSITION",
            "Nav2 lifecycle transition observed",
            node=node_name,
            start_state=message.start_state.label,
            goal_state=message.goal_state.label,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavObserver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
