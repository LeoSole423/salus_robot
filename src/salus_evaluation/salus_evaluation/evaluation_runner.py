"""Thin ROS collector and runner for the pure navigation evaluation domain."""

from __future__ import annotations

import math
import json
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.node import Node
from salus_interfaces.msg import (CmdVelFinal, DriveTelemetry, NavEvent,
                                  NavTelemetry, VehicleCommand)
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .artifacts import write_artifacts
from .gates import GateState, functional_gates, performance_gate
from .metrics import (absolute_goal, arrival_metrics, command_response_sign,
                      command_stage_alignments, expected_turn_from_path,
                      first_divergent_stage, latest_prior,
                      localization_metrics, saturation_intervals,
                      tracking_metrics, trial_data_finite)
from .models import (ExpectedTurn, Pose2D, TimedCommand,
                     TimedControllerStatus, TimedControllerTelemetry,
                     TimedDriveTelemetry, TimedFinalCommand,
                     TimedPose, TimedVehicleCommand)
from .schema import load_scenario


def _stamp(message):
    stamp = message.header.stamp if hasattr(message, "header") else message.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _yaw(quaternion):
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def _timed_odometry(message):
    pose = message.pose.pose
    return TimedPose(_stamp(message), Pose2D(pose.position.x, pose.position.y,
                                             _yaw(pose.orientation)),
                     message.twist.twist.linear.x, message.twist.twist.angular.z)


def _now_s(node):
    return node.get_clock().now().nanoseconds / 1e9


def _finite_float(value):
    """Accept only finite JSON numbers; malformed data stays observable as absent."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    result = float(value)
    return result if math.isfinite(result) else None


def _status_snapshot(stamp_s, payload):
    command = payload.get("command")
    required = {
        "drive_enabled", "estop", "speed_mps", "brake_pct",
        "requested_linear_x_mps", "requested_angular_z_rps",
        "requested_steer_rad", "applied_steer_rad", "steering_limit_used_rad",
        "steer_saturated", "speed_limited", "min_speed_enforced",
    }
    if not isinstance(command, dict) or not required.issubset(command):
        return None
    booleans = (payload.get("fresh"), command.get("drive_enabled"),
                command.get("estop"), command.get("steer_saturated"),
                command.get("speed_limited"), command.get("min_speed_enforced"))
    numeric_keys = required - {
        "drive_enabled", "estop", "steer_saturated", "speed_limited",
        "min_speed_enforced",
    }
    numeric = {key: _finite_float(command[key]) for key in numeric_keys}
    if not all(isinstance(value, bool) for value in booleans) or any(
            value is None for value in numeric.values()):
        return None
    source = payload.get("source", "unknown")
    if not isinstance(source, str) or not (
            isinstance(command["brake_pct"], int) and
            not isinstance(command["brake_pct"], bool)):
        return None
    return TimedControllerStatus(
        stamp_s=stamp_s,
        source=source,
        fresh=payload["fresh"], drive_enabled=command["drive_enabled"],
        estop=command["estop"], speed_mps=numeric["speed_mps"],
        brake_pct=int(numeric["brake_pct"]),
        requested_linear_x_mps=numeric["requested_linear_x_mps"],
        requested_angular_z_rps=numeric["requested_angular_z_rps"],
        requested_steer_rad=numeric["requested_steer_rad"],
        applied_steer_rad=numeric["applied_steer_rad"],
        steering_limit_used_rad=numeric["steering_limit_used_rad"],
        steer_saturated=command["steer_saturated"],
        speed_limited=command["speed_limited"],
        min_speed_enforced=command["min_speed_enforced"],
    )


def _telemetry_snapshot(stamp_s, payload):
    command = payload.get("requested_auto_command")
    limits = payload.get("ackermann_limits")
    required_command = {"speed_mps", "requested_steer_rad", "applied_steer_rad"}
    required_limits = {
        "steering_limit_deg", "operational_steering_limit_deg",
        "effective_steering_limit_deg",
    }
    if (
        not isinstance(command, dict)
        or not isinstance(limits, dict)
        or not required_command.issubset(command)
        or not required_limits.issubset(limits)
    ):
        return None
    numeric = {
        key: _finite_float(command[key]) for key in required_command
    }
    numeric.update({key: _finite_float(limits[key]) for key in required_limits})
    if any(value is None for value in numeric.values()):
        return None
    return TimedControllerTelemetry(
        stamp_s=stamp_s,
        requested_speed_mps=numeric["speed_mps"],
        requested_steer_rad=numeric["requested_steer_rad"],
        applied_steer_rad=numeric["applied_steer_rad"],
        steering_limit_deg=numeric["steering_limit_deg"],
        operational_steering_limit_deg=numeric["operational_steering_limit_deg"],
        effective_steering_limit_deg=numeric["effective_steering_limit_deg"],
    )


def _source_counts(samples):
    counts = {}
    for sample in samples:
        key = str(sample.source)
        counts[key] = counts.get(key, 0) + 1
    return counts


def _value_summary(rows, key):
    values = [row[key] for row in rows if row.get(key) is not None]
    if not values:
        return {"count": 0, "last": None, "min": None, "max": None}
    return {"count": len(values), "last": values[-1], "min": min(values), "max": max(values)}


def _alignment_summary(rows):
    available = [row for row in rows if row.get("available")]
    stale = [row for row in rows if not row.get("available") and
             row.get("alignment_gap_s") is not None]
    divergent = [row for row in available if row.get("divergent")]
    return {
        "total": len(rows), "correlated": len(available),
        "unavailable": len(rows) - len(available), "stale": len(stale),
        "divergent": len(divergent),
        "linear_delta_mps": _value_summary(available, "linear_delta_mps"),
        "angular_delta_rps": _value_summary(available, "angular_delta_rps"),
    }


def _histogram(samples, key):
    counts = {}
    for item in samples:
        value = str(getattr(item, key))
        counts[value] = counts.get(value, 0) + 1
    return counts


def _trial_json_error_counts(errors, goal_stamp_s):
    return {
        source: sum(error_source == source and stamp_s >= goal_stamp_s
                    for error_source, stamp_s in errors)
        for source in ("status", "telemetry")
    }


def _command_chain(raw, safe, final, vehicle, drive, status, telemetry):
    """Build observer-only command-chain evidence and derived causal pairings."""
    final_twist = tuple(
        TimedCommand(item.stamp_s, item.linear_x_mps, item.angular_z_rps, "cmd_vel_final")
        for item in final
    )
    raw_safe = command_stage_alignments(raw, safe)
    safe_final = command_stage_alignments(safe, final_twist)
    translations, applied_measurements = [], []
    for item in vehicle:
        previous, gap_s = latest_prior(final_twist, item.stamp_s)
        translations.append({
            "stage": "twist_to_ackermann",
            "stamp_s": item.stamp_s,
            "alignment_gap_s": gap_s,
            "available": previous is not None,
            "final_linear_x_mps": previous.linear_x_mps if previous else None,
            "final_angular_z_rps": previous.angular_z_rps if previous else None,
            "vehicle_speed_mps": item.speed_mps,
            "vehicle_steering_angle_rad": item.steering_angle_rad,
            "vehicle_source": item.source,
            "vehicle_drive_enabled": item.drive_enabled,
            "vehicle_emergency_stop": item.emergency_stop,
            "vehicle_brake_ratio": item.brake_ratio,
        })
    for item in drive:
        command, command_gap_s = latest_prior(status, item.stamp_s)
        requested, requested_gap_s = latest_prior(telemetry, item.stamp_s)
        applied_measurements.append({
            "stage": "ackermann_to_measured",
            "stamp_s": item.stamp_s,
            "status_alignment_gap_s": command_gap_s,
            "telemetry_alignment_gap_s": requested_gap_s,
            "available": command is not None or requested is not None,
            "status_speed_mps": command.speed_mps if command else None,
            "status_requested_steer_rad": command.requested_steer_rad if command else None,
            "status_applied_steer_rad": command.applied_steer_rad if command else None,
            "status_requested_to_applied_steer_delta_rad": (
                command.applied_steer_rad - command.requested_steer_rad
                if command else None
            ),
            "steer_saturated": command.steer_saturated if command else None,
            "telemetry_requested_speed_mps": requested.requested_speed_mps if requested else None,
            "telemetry_requested_steer_rad": requested.requested_steer_rad if requested else None,
            "telemetry_applied_steer_rad": requested.applied_steer_rad if requested else None,
            "effective_steering_limit_deg": (
                requested.effective_steering_limit_deg if requested else None
            ),
            "speed_mps_measured": item.speed_mps_measured if item.speed_valid else None,
            "steer_rad_measured": item.steer_rad_measured if item.steer_valid else None,
            "status_speed_to_measured_delta_mps": (
                item.speed_mps_measured - command.speed_mps
                if command and item.speed_valid else None
            ),
            "status_applied_to_measured_steer_delta_rad": (
                item.steer_rad_measured - command.applied_steer_rad
                if command and item.steer_valid else None
            ),
            "telemetry_requested_to_measured_speed_delta_mps": (
                item.speed_mps_measured - requested.requested_speed_mps
                if requested and item.speed_valid else None
            ),
            "telemetry_applied_to_measured_steer_delta_rad": (
                item.steer_rad_measured - requested.applied_steer_rad
                if requested and item.steer_valid else None
            ),
            "brake_applied_pct": item.brake_applied_pct,
        })
    return {
        "raw_safe": raw_safe,
        "safe_final": safe_final,
        "twist_to_ackermann": tuple(translations),
        "ackermann_to_measured": tuple(applied_measurements),
        "summary": {
            "first_divergent_stage": first_divergent_stage(raw_safe, safe_final),
            "sample_counts": {
                "cmd_vel": len(raw), "cmd_vel_safe": len(safe),
                "cmd_vel_final": len(final), "vehicle_command": len(vehicle),
                "drive_telemetry": len(drive), "controller_status": len(status),
                "controller_telemetry": len(telemetry),
            },
            "cmd_vel_final": {
                "source_counts": _source_counts(final),
                "brake_sample_count": sum(item.brake_pct > 0 for item in final),
                "brake_pct_histogram": _histogram(final, "brake_pct"),
            },
            "vehicle_command": {
                "source_counts": _source_counts(vehicle),
                "drive_enabled_count": sum(item.drive_enabled for item in vehicle),
                "emergency_stop_count": sum(item.emergency_stop for item in vehicle),
            },
            "steering_saturation": saturation_intervals(status),
            "alignment": {
                "cmd_vel_to_cmd_vel_safe": _alignment_summary(raw_safe),
                "cmd_vel_safe_to_cmd_vel_final": _alignment_summary(safe_final),
                "twist_to_ackermann": {
                    "total": len(translations),
                    "correlated": sum(row["available"] for row in translations),
                    "unavailable": sum(not row["available"] for row in translations),
                },
                "ackermann_to_measured": {
                    "total": len(applied_measurements),
                    "status_unavailable": sum(
                        row["status_alignment_gap_s"] is None or
                        row["status_speed_mps"] is None for row in applied_measurements
                    ),
                    "telemetry_unavailable": sum(
                        row["telemetry_alignment_gap_s"] is None or
                        row["telemetry_requested_speed_mps"] is None
                        for row in applied_measurements
                    ),
                },
            },
            "ackermann": {
                "requested_to_applied_steer_delta_rad": _value_summary(
                    applied_measurements, "status_requested_to_applied_steer_delta_rad"
                ),
                "status_speed_to_measured_delta_mps": _value_summary(
                    applied_measurements, "status_speed_to_measured_delta_mps"
                ),
                "status_applied_to_measured_steer_delta_rad": _value_summary(
                    applied_measurements, "status_applied_to_measured_steer_delta_rad"
                ),
                "telemetry_requested_to_measured_speed_delta_mps": _value_summary(
                    applied_measurements,
                    "telemetry_requested_to_measured_speed_delta_mps",
                ),
                "telemetry_applied_to_measured_steer_delta_rad": _value_summary(
                    applied_measurements,
                    "telemetry_applied_to_measured_steer_delta_rad",
                ),
            },
            "ackermann_limits": {
                "steering_limit_deg": _value_summary(
                    [vars(item) for item in telemetry], "steering_limit_deg"
                ),
                "operational_steering_limit_deg": _value_summary(
                    [vars(item) for item in telemetry], "operational_steering_limit_deg"
                ),
                "effective_steering_limit_deg": _value_summary(
                    [vars(item) for item in telemetry], "effective_steering_limit_deg"
                ),
            },
        },
    }


class EvaluationRunner(Node):
    """Observe standard ROS topics and persist a reproducible trial result."""

    def __init__(self):
        super().__init__("navigation_evaluation")
        self.declare_parameter("scenario", "")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("mode", "run")
        self.declare_parameter("goal_tolerance_m", 1.2)
        self.declare_parameter("precision_target_m", 0.25)
        self.declare_parameter("observe_timeout_s", 90.0)
        self.scenario_path = str(self.get_parameter("scenario").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.mode = str(self.get_parameter("mode").value)
        self.tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self.precision_target = float(self.get_parameter("precision_target_m").value)
        self.timeout_s = float(self.get_parameter("observe_timeout_s").value)
        if not self.output_dir:
            raise ValueError("output_dir is required")
        if self.mode not in ("run", "observe"):
            raise ValueError("mode must be run or observe")
        if self.mode == "run" and not self.scenario_path:
            raise ValueError("scenario is required in run mode")
        if self.tolerance <= 0.0 or self.precision_target <= 0.0:
            raise ValueError("arrival tolerances must be positive")
        self.global_poses, self.raw_poses, self.local_poses = [], [], []
        self.commands, self.safe_commands, self.final_commands = [], [], []
        self.vehicle_commands, self.drive_telemetry = [], []
        self.controller_status, self.controller_telemetry = [], []
        self.controller_json_errors = []
        self.plans, self.events = [], []
        self.goal = None
        self.start_pose = None
        self.goal_sent_s = None
        self.success_s = None
        self.expected_turn = ExpectedTurn.ANY
        self.reverse_allowed = False
        self.terminal_status = None
        self.terminal_received_s = None
        self.last_marker_s = None
        self.telemetry = None
        self.goal_event_baseline = None
        self._finished = False
        self.exit_code = 1
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.markers = self.create_publisher(MarkerArray, "/navigation_evaluation/markers", 10)
        self.create_subscription(Odometry, "/odometry/global", self._global, 50)
        self.create_subscription(Odometry, "/odom_raw", self._raw, 50)
        self.create_subscription(Odometry, "/odometry/local", self._local, 50)
        self.create_subscription(Twist, "/cmd_vel", self._command, 50)
        self.create_subscription(Twist, "/cmd_vel_safe", self._safe_command, 50)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self._final_command, 50)
        self.create_subscription(
            VehicleCommand, "/vehicle/command_shadow", self._vehicle_command, 50
        )
        self.create_subscription(
            DriveTelemetry, "/controller/drive_telemetry", self._drive_telemetry, 50
        )
        self.create_subscription(String, "/controller/status", self._controller_status, 20)
        self.create_subscription(
            String, "/controller/telemetry", self._controller_telemetry, 20
        )
        self.create_subscription(NavPath, "/plan", self._plan, 10)
        self.create_subscription(NavEvent, "/nav_command_server/events", self._event, 20)
        self.create_subscription(
            NavTelemetry, "/nav_command_server/telemetry", self._telemetry, 20
        )
        self.create_subscription(PoseStamped, "/goal_pose", self._observed_goal, 10)
        self.create_timer(0.1, self._tick)

    def _global(self, message):
        self.global_poses.append(_timed_odometry(message))

    def _raw(self, message):
        self.raw_poses.append(_timed_odometry(message))

    def _local(self, message):
        self.local_poses.append(_timed_odometry(message))

    def _command(self, message):
        self.commands.append(TimedCommand(_now_s(self), message.linear.x, message.angular.z))

    def _safe_command(self, message):
        self.safe_commands.append(
            TimedCommand(_now_s(self), message.linear.x, message.angular.z, "cmd_vel_safe")
        )

    def _final_command(self, message):
        self.final_commands.append(TimedFinalCommand(
            _now_s(self), message.twist.linear.x, message.twist.angular.z,
            int(message.brake_pct), int(message.source),
        ))

    def _vehicle_command(self, message):
        self.vehicle_commands.append(TimedVehicleCommand(
            _stamp(message), int(message.source), bool(message.drive_enabled),
            bool(message.emergency_stop), float(message.brake_ratio),
            float(message.drive.speed), float(message.drive.steering_angle),
        ))

    def _drive_telemetry(self, message):
        self.drive_telemetry.append(TimedDriveTelemetry(
            _stamp(message), bool(message.ready), bool(message.fresh),
            bool(message.drive_enabled), bool(message.estop),
            bool(message.speed_valid), bool(message.steer_valid),
            str(message.control_source), float(message.speed_mps_measured),
            math.radians(float(message.steer_deg_measured)),
            int(message.brake_applied_pct),
        ))

    def _controller_status(self, message):
        stamp_s = _now_s(self)
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.controller_json_errors.append(("status", stamp_s))
            return
        snapshot = _status_snapshot(stamp_s, payload) if isinstance(payload, dict) else None
        if snapshot is None:
            self.controller_json_errors.append(("status", stamp_s))
        else:
            self.controller_status.append(snapshot)

    def _controller_telemetry(self, message):
        stamp_s = _now_s(self)
        try:
            payload = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.controller_json_errors.append(("telemetry", stamp_s))
            return
        snapshot = (
            _telemetry_snapshot(stamp_s, payload)
            if isinstance(payload, dict) else None
        )
        if snapshot is None:
            self.controller_json_errors.append(("telemetry", stamp_s))
        else:
            self.controller_telemetry.append(snapshot)

    def _plan(self, message):
        points = tuple(Pose2D(item.pose.position.x, item.pose.position.y,
                              _yaw(item.pose.orientation))
                       for item in message.poses)
        if points:
            self.plans.append(points)
            if (self.mode == "observe" and self.goal is not None and
                    self.expected_turn == ExpectedTurn.ANY and
                    self.start_pose is not None):
                try:
                    self.expected_turn = expected_turn_from_path(
                        self.start_pose, points
                    )
                except ValueError:
                    pass

    def _event(self, message):
        self.events.append((message.code, _stamp(message)))
        if message.code == "GOAL_RESULT_SUCCEEDED":
            self.success_s, self.terminal_status = _stamp(message), GoalStatus.STATUS_SUCCEEDED
        elif message.code in ("GOAL_RESULT_ABORTED", "GOAL_CANCELLED"):
            self.terminal_status = (GoalStatus.STATUS_ABORTED if message.code.endswith("ABORTED")
                                    else GoalStatus.STATUS_CANCELED)
        if self.terminal_status is not None:
            self.terminal_received_s = self.get_clock().now().nanoseconds / 1e9

    def _telemetry(self, message):
        self.telemetry = message
        terminal = int(message.nav_result_status)
        is_new_result = (
            self.goal_event_baseline is not None and
            int(message.nav_result_event_id) > self.goal_event_baseline
        )
        if is_new_result and terminal in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                GoalStatus.STATUS_CANCELED):
            self.terminal_status = terminal
            now = self.get_clock().now().nanoseconds / 1e9
            self.terminal_received_s = now
            if terminal == GoalStatus.STATUS_SUCCEEDED and self.success_s is None:
                self.success_s = now

    def _observed_goal(self, message):
        if self.mode != "observe" or self.goal is not None:
            return
        if message.header.frame_id.lstrip("/") != "map":
            self.get_logger().warn("ignoring evaluation goal outside map frame")
            return
        self.goal = Pose2D(message.pose.position.x, message.pose.position.y,
                           _yaw(message.pose.orientation))
        self.start_pose = self.global_poses[-1].pose if self.global_poses else None
        self.terminal_status = None
        self.terminal_received_s = None
        self.goal_event_baseline = (
            int(self.telemetry.nav_result_event_id) if self.telemetry else None
        )
        self.goal_sent_s = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info("observing RViz goal")

    def _send_scenario_goal(self):
        scenario = load_scenario(self.scenario_path)
        if len(scenario.goals) != 1:
            raise ValueError("v1 runner supports exactly one goal per trial")
        goal_spec = scenario.goals[0]
        self.goal = absolute_goal(scenario.spawn, goal_spec)
        self.expected_turn = goal_spec.expected_turn
        self.reverse_allowed = goal_spec.reverse_allowed
        self.timeout_s = goal_spec.timeout_s
        self.terminal_status = None
        self.terminal_received_s = None
        self.goal_event_baseline = int(self.telemetry.nav_result_event_id)
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
        if (self.mode == "run" and self.goal is None and self.global_poses and
                self.telemetry is not None):
            self._send_scenario_goal()
            return
        if self.goal is None or self.goal_sent_s is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last_marker_s is None or now - self.last_marker_s >= 0.5:
            self._publish_markers()
            self.last_marker_s = now
        success_window_complete = (self.terminal_status == GoalStatus.STATUS_SUCCEEDED and
                                   self.terminal_received_s is not None and
                                   now - self.terminal_received_s >= 1.0)
        failed_terminal = self.terminal_status in (GoalStatus.STATUS_ABORTED,
                                                   GoalStatus.STATUS_CANCELED)
        if success_window_complete or failed_terminal or now - self.goal_sent_s > self.timeout_s:
            self._finish("timeout" if self.terminal_status is None else "terminal")

    def _publish_markers(self):
        markers = MarkerArray()
        path = Marker()
        path.header.frame_id = "map"
        path.ns, path.id = "evaluation", 0
        path.type, path.action = Marker.LINE_STRIP, Marker.ADD
        path.scale.x, path.color.a, path.color.g = .03, 1.0, 1.0
        for item in self.global_poses:
            path.points.append(Point(x=item.pose.x_m, y=item.pose.y_m, z=.05))
        markers.markers.append(path)
        self.markers.publish(markers)

    def _finish(self, reason):
        self._finished = True
        global_poses = tuple(
            item for item in self.global_poses if item.stamp_s >= self.goal_sent_s
        )
        raw_poses = tuple(item for item in self.raw_poses if item.stamp_s >= self.goal_sent_s)
        local_poses = tuple(item for item in self.local_poses if item.stamp_s >= self.goal_sent_s)
        commands = tuple(item for item in self.commands if item.stamp_s >= self.goal_sent_s)
        safe_commands = tuple(
            item for item in self.safe_commands if item.stamp_s >= self.goal_sent_s
        )
        final_commands = tuple(
            item for item in self.final_commands if item.stamp_s >= self.goal_sent_s
        )
        vehicle_commands = tuple(
            item for item in self.vehicle_commands if item.stamp_s >= self.goal_sent_s
        )
        drive_telemetry = tuple(
            item for item in self.drive_telemetry if item.stamp_s >= self.goal_sent_s
        )
        controller_status = tuple(
            item for item in self.controller_status if item.stamp_s >= self.goal_sent_s
        )
        controller_telemetry = tuple(
            item for item in self.controller_telemetry if item.stamp_s >= self.goal_sent_s
        )
        controller_json_errors = _trial_json_error_counts(
            self.controller_json_errors, self.goal_sent_s
        )
        plan = self.plans[-1] if self.plans else ()
        finite = trial_data_finite(
            self.goal, (global_poses, raw_poses, local_poses), commands, plan
        )
        metrics, arrival, localization = None, None, None
        signs = command_response_sign(commands, raw_poses) if raw_poses else None
        errors = []
        try:
            if global_poses and plan:
                metrics = tracking_metrics(global_poses, plan)
            if global_poses and self.goal:
                arrival = arrival_metrics(global_poses, self.goal, self.tolerance, self.success_s)
            if raw_poses and global_poses:
                localization = localization_metrics(raw_poses, global_poses)
        except ValueError as exc:
            errors.append(str(exc))
        gates = functional_gates(
            finite_data=finite and not errors, plan_present=bool(plan),
            terminal_success=self.terminal_status == GoalStatus.STATUS_SUCCEEDED,
            final_distance_m=arrival.final_distance_m if arrival else float("inf"),
            tolerance_m=self.tolerance, sign_metrics=signs or command_response_sign((), ()),
            reverse_observed=any(command.linear_x_mps < -.01 for command in commands),
            reverse_allowed=self.reverse_allowed, expected_turn=self.expected_turn,
            require_turn_expectation=self.mode == "observe",
        )
        precision = {
            "target_m": self.precision_target,
            "final_error_m": arrival.final_distance_m if arrival else None,
            "target_met": (
                arrival is not None and
                arrival.final_distance_m <= self.precision_target
            ),
            "state": "calibrating",
        }
        command_chain = _command_chain(
            commands, safe_commands, final_commands, vehicle_commands,
            drive_telemetry, controller_status, controller_telemetry,
        )
        summary = {"schema_version": 2, "reason": reason,
                   "terminal_status": self.terminal_status,
                   "goal": self.goal, "metrics": metrics, "arrival": arrival,
                   "operational_tolerance_m": self.tolerance,
                   "precision": precision,
                   "localization": localization, "sign": signs, "gates": gates,
                   "performance": [performance_gate(
                       "cross_track_p95_m",
                       metrics.cross_track_p95_m if metrics else float("inf"),
                   )],
                   "errors": errors, "replans": max(0, len(self.plans) - 1),
                   "command_chain": command_chain["summary"],
                   "controller_json_errors": controller_json_errors}
        manifest = {
            "schema_version": 2, "goal_stamp_s": self.goal_sent_s,
            "mode": self.mode, "scenario": self.scenario_path,
            "streams": [
                "odometry_global", "odometry_raw", "odometry_local", "commands",
                "commands_safe", "commands_final", "vehicle_commands",
                "drive_telemetry", "controller_status", "controller_telemetry",
                "command_chain_alignment",
            ],
            "topics": [
                "/plan", "/cmd_vel", "/cmd_vel_safe", "/cmd_vel_final",
                "/vehicle/command_shadow", "/controller/drive_telemetry",
                "/controller/status", "/controller/telemetry", "/odom_raw",
                "/odometry/local", "/odometry/global",
            ],
        }
        streams = {"odometry_global": global_poses, "odometry_raw": raw_poses,
                   "odometry_local": local_poses, "commands": commands,
                   "commands_safe": safe_commands, "commands_final": final_commands,
                   "vehicle_commands": vehicle_commands,
                   "drive_telemetry": drive_telemetry,
                   "controller_status": controller_status,
                   "controller_telemetry": controller_telemetry,
                   "command_chain_alignment": (
                       command_chain["raw_safe"] + command_chain["safe_final"]
                       + command_chain["twist_to_ackermann"]
                       + command_chain["ackermann_to_measured"]
                   )}
        write_artifacts(self.output_dir, manifest, summary, streams)
        self.exit_code = int(any(gate.state == GateState.FAIL for gate in gates))
        self._publish_markers()
        self.get_logger().info(f"evaluation complete: {Path(self.output_dir) / 'summary.json'}")


def main():
    rclpy.init()
    node = EvaluationRunner()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node)
    finally:
        exit_code = node.exit_code
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
