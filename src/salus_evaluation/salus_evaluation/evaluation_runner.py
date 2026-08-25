"""Thin ROS collector and runner for the pure navigation evaluation domain."""

from __future__ import annotations

import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from salus_interfaces.msg import NavEvent, NavTelemetry
from visualization_msgs.msg import Marker, MarkerArray

from .artifacts import write_artifacts
from .gates import functional_gates, performance_gate
from .metrics import (absolute_goal, arrival_metrics, command_response_sign,
                      localization_metrics, tracking_metrics)
from .models import Pose2D, TimedCommand, TimedPose
from .schema import load_scenario


def _stamp(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _yaw(quaternion):
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def _timed_odometry(message):
    pose = message.pose.pose
    return TimedPose(_stamp(message), Pose2D(pose.position.x, pose.position.y,
                                             _yaw(pose.orientation)),
                     message.twist.twist.linear.x, message.twist.twist.angular.z)


class EvaluationRunner(Node):
    """Observe standard ROS topics and persist a reproducible trial result."""

    def __init__(self):
        super().__init__("navigation_evaluation")
        self.declare_parameter("scenario", "")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("mode", "run")
        self.declare_parameter("goal_tolerance_m", 0.25)
        self.declare_parameter("observe_timeout_s", 90.0)
        self.scenario_path = str(self.get_parameter("scenario").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.mode = str(self.get_parameter("mode").value)
        self.tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.timeout_s = float(self.get_parameter("observe_timeout_s").value)
        if not self.output_dir:
            raise ValueError("output_dir is required")
        self.global_poses, self.raw_poses, self.local_poses = [], [], []
        self.commands, self.plans, self.events = [], [], []
        self.goal = None
        self.goal_sent_s = None
        self.success_s = None
        self.terminal_status = None
        self._finished = False
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.markers = self.create_publisher(MarkerArray, "/navigation_evaluation/markers", 10)
        self.create_subscription(Odometry, "/odometry/global", self._global, 50)
        self.create_subscription(Odometry, "/odom_raw", self._raw, 50)
        self.create_subscription(Odometry, "/odometry/local", self._local, 50)
        self.create_subscription(Twist, "/cmd_vel", self._command, 50)
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_subscription(NavEvent, "/nav_command_server/events", self._event, 20)
        self.create_subscription(NavTelemetry, "/nav_command_server/telemetry", self._telemetry, 20)
        self.create_subscription(PoseStamped, "/goal_pose", self._observed_goal, 10)
        self.create_timer(0.1, self._tick)

    def _global(self, message):
        self.global_poses.append(_timed_odometry(message))

    def _raw(self, message):
        self.raw_poses.append(_timed_odometry(message))

    def _local(self, message):
        self.local_poses.append(_timed_odometry(message))

    def _command(self, message):
        now = self.get_clock().now().nanoseconds / 1e9
        self.commands.append(TimedCommand(now, message.linear.x, message.angular.z))

    def _plan(self, message):
        points = tuple(Pose2D(item.pose.position.x, item.pose.position.y,
                               _yaw(item.pose.orientation)) for item in message.poses)
        if points:
            self.plans.append(points)

    def _event(self, message):
        self.events.append((message.code, _stamp(message)))
        if message.code == "GOAL_RESULT_SUCCEEDED":
            self.success_s, self.terminal_status = _stamp(message), GoalStatus.STATUS_SUCCEEDED
        elif message.code in ("GOAL_RESULT_ABORTED", "GOAL_CANCELLED"):
            self.terminal_status = (GoalStatus.STATUS_ABORTED if message.code.endswith("ABORTED")
                                    else GoalStatus.STATUS_CANCELED)

    def _telemetry(self, message):
        if message.nav_result_status in (GoalStatus.STATUS_SUCCEEDED,
                                         GoalStatus.STATUS_ABORTED,
                                         GoalStatus.STATUS_CANCELED):
            self.terminal_status = int(message.nav_result_status)
            if self.terminal_status == GoalStatus.STATUS_SUCCEEDED and self.success_s is None:
                self.success_s = self.get_clock().now().nanoseconds / 1e9

    def _observed_goal(self, message):
        if self.mode != "observe" or self.goal is not None:
            return
        if message.header.frame_id.lstrip("/") != "map":
            self.get_logger().warn("ignoring evaluation goal outside map frame")
            return
        self.goal = Pose2D(message.pose.position.x, message.pose.position.y,
                           _yaw(message.pose.orientation))
        self.goal_sent_s = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info("observing RViz goal")

    def _send_scenario_goal(self):
        scenario = load_scenario(self.scenario_path)
        if len(scenario.goals) != 1:
            raise ValueError("v1 runner supports exactly one goal per trial")
        self.goal = absolute_goal(scenario.spawn, scenario.goals[0])
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y = self.goal.x_m, self.goal.y_m
        message.pose.orientation.z = math.sin(self.goal.yaw_rad / 2.0)
        message.pose.orientation.w = math.cos(self.goal.yaw_rad / 2.0)
        self.goal_pub.publish(message)
        self.goal_sent_s = self.get_clock().now().nanoseconds / 1e9

    def _tick(self):
        if self._finished:
            return
        if self.mode == "run" and self.goal is None and self.global_poses:
            self._send_scenario_goal()
            return
        if self.goal is None or self.goal_sent_s is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.terminal_status is not None or now - self.goal_sent_s > self.timeout_s:
            self._finish("timeout" if self.terminal_status is None else "terminal")

    def _publish_markers(self):
        markers = MarkerArray()
        path = Marker()
        path.header.frame_id, path.ns, path.id, path.type, path.action = "map", "evaluation", 0, Marker.LINE_STRIP, Marker.ADD
        path.scale.x, path.color.a, path.color.g = .03, 1.0, 1.0
        for item in self.global_poses:
            path.points.append(Point(x=item.pose.x_m, y=item.pose.y_m, z=.05))
        markers.markers.append(path)
        self.markers.publish(markers)

    def _finish(self, reason):
        self._finished = True
        finite = all(math.isfinite(value) for collection in (self.global_poses, self.raw_poses, self.local_poses)
                     for item in collection for value in (item.stamp_s, item.pose.x_m, item.pose.y_m, item.pose.yaw_rad))
        plan = self.plans[-1] if self.plans else ()
        metrics, arrival, localization, signs = None, None, None, command_response_sign(self.commands, self.raw_poses) if self.raw_poses else None
        errors = []
        try:
            if self.global_poses and plan:
                metrics = tracking_metrics(self.global_poses, plan)
            if self.global_poses and self.goal:
                arrival = arrival_metrics(self.global_poses, self.goal, self.tolerance, self.success_s)
            if self.raw_poses and self.global_poses:
                localization = localization_metrics(self.raw_poses, self.global_poses)
        except ValueError as exc:
            errors.append(str(exc))
        gates = functional_gates(
            finite_data=finite and not errors, plan_present=bool(plan),
            terminal_success=self.terminal_status == GoalStatus.STATUS_SUCCEEDED,
            final_distance_m=arrival.final_distance_m if arrival else float("inf"),
            tolerance_m=self.tolerance, sign_metrics=signs or command_response_sign((), ()),
            reverse_observed=any(command.linear_x_mps < -.01 for command in self.commands),
            reverse_allowed=False,
        )
        summary = {"schema_version": 1, "reason": reason, "terminal_status": self.terminal_status,
                   "goal": self.goal, "metrics": metrics, "arrival": arrival,
                   "localization": localization, "sign": signs, "gates": gates,
                   "performance": [performance_gate("cross_track_p95_m", metrics.cross_track_p95_m if metrics else float("inf"))],
                   "errors": errors, "replans": max(0, len(self.plans) - 1)}
        manifest = {"schema_version": 1, "started_unix_s": self.goal_sent_s,
                    "mode": self.mode, "scenario": self.scenario_path, "topics": ["/plan", "/cmd_vel", "/odom_raw", "/odometry/local", "/odometry/global"]}
        streams = {"odometry_global": self.global_poses, "odometry_raw": self.raw_poses,
                   "odometry_local": self.local_poses, "commands": self.commands}
        write_artifacts(self.output_dir, manifest, summary, streams)
        self._publish_markers()
        self.get_logger().info(f"evaluation complete: {Path(self.output_dir) / 'summary.json'}")
        rclpy.shutdown()


def main():
    rclpy.init()
    node = EvaluationRunner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
