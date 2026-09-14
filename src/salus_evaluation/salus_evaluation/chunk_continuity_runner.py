"""Observer-only Nav2 experiment for continuity at a wide route boundary."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from salus_navigation.route_geometry import path_geometry_metrics

from .artifacts import write_artifacts
from .evaluation_runner import _status_snapshot
from .gates import functional_gates, performance_gate
from .metrics import (angle_delta, arrival_metrics, command_response_sign,
                      localization_metrics, saturation_intervals,
                      steering_margin_summary, tracking_metrics)
from .models import ExpectedTurn, Pose2D, TimedPose


POLICIES = {
    "terminal_incoming", "legacy_outgoing", "shared_tangent", "lookahead",
}
BOUNDARY_LOOKAHEAD_M = 2.0


def _yaw(quaternion):
    return math.atan2(2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                      1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def _stamp(message):
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) / 1e9


def _pose_message(node, point, yaw):
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x, message.pose.position.y = point
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    return message


def _rotate(origin, forward, lateral, yaw):
    return (origin[0] + forward * math.cos(yaw) - lateral * math.sin(yaw),
            origin[1] + forward * math.sin(yaw) + lateral * math.cos(yaw))


def _curvature(points, center_index):
    values = []
    low, high = max(1, center_index - 6), min(len(points) - 1, center_index + 6)
    for index in range(low, high):
        first, middle, last = points[index - 1:index + 2]
        first_heading = math.atan2(middle[1] - first[1], middle[0] - first[0])
        second_heading = math.atan2(last[1] - middle[1], last[0] - middle[0])
        scale = (math.hypot(middle[0] - first[0], middle[1] - first[1])
                 + math.hypot(last[0] - middle[0], last[1] - middle[1])) / 2.0
        if scale > 1e-9:
            values.append(abs(angle_delta(second_heading, first_heading)) / scale)
    return max(values, default=None), max(
        (abs(angle_delta(
            math.atan2(points[index + 1][1] - points[index][1],
                       points[index + 1][0] - points[index][0]),
            math.atan2(points[index][1] - points[index - 1][1],
                       points[index][0] - points[index - 1][0])))
         for index in range(low, high)), default=None)


def _boundary_metrics(first, second, boundary):
    if len(first) < 2 or len(second) < 2:
        return {"boundary_heading_jump_rad": None,
                "boundary_max_curvature_per_m": None,
                "boundary_max_heading_step_rad": None}
    incoming = math.atan2(first[-1][1] - first[-2][1], first[-1][0] - first[-2][0])
    outgoing = math.atan2(second[1][1] - second[0][1], second[1][0] - second[0][0])
    combined = tuple(first) + tuple(second)
    nearest = min(range(len(combined)),
                  key=lambda index: math.hypot(combined[index][0] - boundary[0],
                                               combined[index][1] - boundary[1]))
    curvature, heading_step = _curvature(combined, min(max(1, nearest), len(combined) - 2))
    return {
        "boundary_incoming_realized_rad": incoming,
        "boundary_outgoing_realized_rad": outgoing,
        "boundary_heading_jump_rad": abs(angle_delta(outgoing, incoming)),
        "boundary_max_curvature_per_m": curvature,
        "boundary_max_heading_step_rad": heading_step,
    }


class ChunkContinuityRunner(Node):
    """Run two sequential Nav2 chunks while only observing control outputs."""

    def __init__(self):
        super().__init__("navigation_chunk_continuity")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("chunk_policy", "")
        self.declare_parameter("scenario", "")
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.policy = str(self.get_parameter("chunk_policy").value)
        if not self.output_dir or self.policy not in POLICIES:
            raise ValueError("output_dir and an approved chunk_policy are required")
        self.odom = []
        self.raw_odom = []
        self.plans = []
        self.statuses = []
        self.plan_action = ActionClient(self, NavigateThroughPoses,
                                        "/navigate_through_poses")
        self.create_subscription(Odometry, "/odometry/global", self._odom, 50)
        self.create_subscription(Odometry, "/odom_raw", self._raw, 50)
        self.create_subscription(NavPath, "/plan", self._plan, 20)
        self.create_subscription(String, "/controller/status", self._status, 20)
        self.started_at = None
        self.origin = None
        self.reference = None
        self.boundary = None
        self.incoming_yaw = None
        self.outgoing_yaw = None
        self.goals = None
        self.first_result = None
        self.second_result = None
        self.first_plan = ()
        self.second_plan = ()
        self.exit_code = 1
        self.done = False

    def _odom(self, message):
        pose = message.pose.pose
        item = TimedPose(_stamp(message), Pose2D(pose.position.x, pose.position.y,
                                                 _yaw(pose.orientation)),
                         message.twist.twist.linear.x, message.twist.twist.angular.z)
        self.odom.append(item)
        if self.origin is None:
            self.origin = (item.pose.x_m, item.pose.y_m, item.pose.yaw_rad)

    def _raw(self, message):
        pose = message.pose.pose
        self.raw_odom.append(TimedPose(
            _stamp(message), Pose2D(pose.position.x, pose.position.y,
                                    _yaw(pose.orientation)),
            message.twist.twist.linear.x, message.twist.twist.angular.z))

    def _plan(self, message):
        points = tuple((float(item.pose.position.x), float(item.pose.position.y))
                       for item in message.poses)
        if points:
            self.plans.append(points)

    def _status(self, message):
        try:
            payload = __import__("json").loads(message.data)
        except (TypeError, ValueError):
            return
        snapshot = _status_snapshot(self.get_clock().now().nanoseconds / 1e9, payload)
        if snapshot is not None:
            self.statuses.append(snapshot)

    def _wait(self, predicate, timeout_s, label):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not predicate():
            if time.monotonic() >= deadline:
                raise RuntimeError(label)
            rclpy.spin_once(self, timeout_sec=0.1)

    def _send(self, points, yaws, label):
        goal = NavigateThroughPoses.Goal()
        goal.poses = [_pose_message(self, point, yaw)
                      for point, yaw in zip(points, yaws)]
        future = self.plan_action.send_goal_async(goal)
        self._wait(future.done, 15.0, f"{label} goal request timed out")
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label} goal rejected")
        result = handle.get_result_async()
        self._wait(result.done, 100.0, f"{label} action timed out")
        return result.result().status

    def _prepare(self):
        self._wait(lambda: self.origin is not None and self.plan_action.server_is_ready(),
                   90.0, "Nav2 readiness timed out")
        x, y, yaw = self.origin
        self.reference = [_rotate((x, y), 4.0, 0.0, yaw),
                          _rotate((x, y), 8.0, 0.0, yaw),
                          _rotate((x, y), 8.0, 8.0, yaw)]
        self.boundary = self.reference[1]
        lookahead = _rotate(self.boundary, 2.0, 0.0, yaw + math.pi / 2.0)
        self.incoming_yaw, self.outgoing_yaw = yaw, yaw + math.pi / 2.0
        shared = self.incoming_yaw + math.pi / 4.0
        terminal = {
            "terminal_incoming": self.incoming_yaw,
            "legacy_outgoing": self.outgoing_yaw,
            "shared_tangent": shared,
            "lookahead": self.incoming_yaw,
        }[self.policy]
        first_points = [self.reference[0], self.reference[1]]
        first_yaws = [self.incoming_yaw, terminal]
        if self.policy == "lookahead":
            first_points.append(lookahead)
            first_yaws.append(self.outgoing_yaw)
        self.goals = (first_points, first_yaws, [self.reference[2]], [self.outgoing_yaw])
        self._wait(lambda: len(self.odom) >= 2, 5.0, "odometry did not become progressive")

    def run(self):
        self._prepare()
        self.started_at = self.get_clock().now().nanoseconds / 1e9
        plan_count = len(self.plans)
        self.first_result = self._send(*self.goals[:2], "first chunk")
        self._wait(lambda: len(self.plans) > plan_count, 10.0,
                   "first chunk produced no plan")
        self.first_plan = self.plans[-1]
        plan_count = len(self.plans)
        if self.first_result == GoalStatus.STATUS_SUCCEEDED:
            self.second_result = self._send(*self.goals[2:], "second chunk")
            self._wait(lambda: len(self.plans) > plan_count, 10.0,
                       "second chunk produced no plan")
            self.second_plan = self.plans[-1]
        self._finish()

    def _finish(self):
        poses = tuple(item for item in self.odom if item.stamp_s >= self.started_at)
        raw = tuple(item for item in self.raw_odom if item.stamp_s >= self.started_at)
        first = self.first_plan
        second = self.second_plan
        combined = tuple(first) + tuple(second)
        geometry = path_geometry_metrics(combined, self.reference) if len(combined) >= 2 else None
        boundary = _boundary_metrics(first, second, self.boundary)
        statuses = tuple(item for item in self.statuses if item.stamp_s >= self.started_at)
        tracking = (
            tracking_metrics(poses, tuple(Pose2D(x, y, 0.0) for x, y in combined))
            if poses and len(combined) >= 2 else None
        )
        goal = Pose2D(*self.reference[2], self.outgoing_yaw)
        arrival = arrival_metrics(poses, goal, 1.2) if poses else None
        finite = all(math.isfinite(value) for item in poses
                     for value in (item.stamp_s, item.pose.x_m, item.pose.y_m,
                                   item.pose.yaw_rad))
        signs = command_response_sign((), poses)
        gates = functional_gates(
            finite_data=finite and bool(combined), plan_present=bool(combined),
            terminal_success=(self.first_result == GoalStatus.STATUS_SUCCEEDED and
                              self.second_result == GoalStatus.STATUS_SUCCEEDED),
            final_distance_m=arrival.final_distance_m if arrival else float("inf"),
            tolerance_m=1.2, sign_metrics=signs, reverse_observed=any(
                item.linear_x_mps < -0.01 for item in poses), reverse_allowed=False,
            expected_turn=ExpectedTurn.ANY,
            require_turn_expectation=False,
        )
        saturation = saturation_intervals(statuses)
        margin = steering_margin_summary(statuses)
        summary = {
            "schema_version": 2,
            "reason": "chunk_continuity",
            "terminal_status": self.second_result or self.first_result,
            "goal": goal,
            "metrics": tracking,
            "arrival": arrival,
            "operational_tolerance_m": 1.2,
            "precision": {"target_m": 0.25, "state": "calibrating",
                          "final_error_m": arrival.final_distance_m if arrival else None},
            "localization": localization_metrics(raw, poses) if raw and poses else None,
            "sign": signs,
            "gates": gates,
            "performance": [performance_gate(
                "boundary_heading_jump_rad",
                boundary.get("boundary_heading_jump_rad") or float("inf"))],
            "errors": [],
            "replans": max(0, len(self.plans) - 2),
            "command_chain": {"steering_saturation": saturation,
                              "steering_margin": margin},
            "chunk_continuity": {
                "policy": self.policy,
                "length_m": geometry.length_m if geometry else None,
                "direct_distance_m": geometry.direct_distance_m if geometry else None,
                "detour_ratio": geometry.detour_ratio if geometry else None,
                "max_deviation_m": geometry.max_deviation_m if geometry else None,
                "self_intersections": geometry.self_intersections if geometry else None,
                **boundary,
                "steering_saturation_intervals": saturation["interval_count"],
                "first_chunk_status": self.first_result,
                "second_chunk_status": self.second_result,
                "plan_count": len(self.plans),
                "explicit_yaws_changed": False,
                "lookahead_synthetic": self.policy == "lookahead",
            },
        }
        manifest = {
            "schema_version": 2, "mode": "chunk_continuity",
            "policy": self.policy, "scenario": "wide_90deg_boundary",
            "geometry": {"leg_length_m": 8.0, "lookahead_m": BOUNDARY_LOOKAHEAD_M,
                         "boundary_inside_wide_turn": True},
            "planner_contract": {"motion_model_for_search": "DUBIN",
                                 "minimum_turning_radius_m": 4.0},
            "topics": ["/plan", "/odometry/global", "/odom_raw", "/controller/status"],
            "streams": ["odometry_global", "odometry_raw", "controller_status"],
        }
        write_artifacts(self.output_dir, manifest, summary, {
            "odometry_global": poses, "odometry_raw": raw, "controller_status": statuses,
        })
        self.exit_code = int(any(item.state.value == "fail" for item in gates))
        self.done = True


def main():
    rclpy.init()
    node = ChunkContinuityRunner()
    try:
        node.run()
        return node.exit_code
    except Exception as exc:
        node.get_logger().error(str(exc))
        write_artifacts(node.output_dir, {
            "schema_version": 2, "mode": "chunk_continuity", "policy": node.policy,
        }, {"schema_version": 2, "reason": "setup_failure", "errors": [str(exc)],
            "chunk_continuity": {"policy": node.policy}}, {})
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
