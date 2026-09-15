"""Evaluation-only isolation of a future hard checkpoint in Nav2 requests."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node

from .artifacts import write_artifacts
from .chunk_continuity_runner import _finite_pose, _plan_metrics, _point_path, _stamp


P0_REPLAY = (
    Path(get_package_share_directory("salus_evaluation"))
    / "config" / "replays" / "issue244_real_waypoint_boundary_p0.json"
)
ARMS = {
    "current_two_key": "CURRENT_TWO_KEY",
    "immediate_key_only_same_yaw": "IMMEDIATE_KEY_ONLY_SAME_YAW",
    "one_pose_ntp": "ONE_POSE_NTP",
    "one_pose_navigate_to_pose": "ONE_POSE_NTPOSE",
}


def _yaw(quaternion):
    """Extract planar yaw from a quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def _pose_dict(pose):
    """Serialize a finite map pose sample."""
    if pose is None:
        return None
    return {
        "stamp_s": pose["stamp_s"],
        "x_m": pose["x_m"],
        "y_m": pose["y_m"],
        "yaw_rad": pose["yaw_rad"],
    }


def _load_fixture():
    """Load the frozen P0 capture without regenerating any route geometry."""
    payload = json.loads(P0_REPLAY.read_text(encoding="utf-8"))
    request = payload["request"]
    poses = tuple(
        ((float(x), float(y)), math.radians(float(yaw)))
        for (x, y), yaw in zip(request["poses_xy"], request["yaws_deg"])
    )
    if payload["synthetic_count"] != 0:
        raise ValueError("P0 fixture must contain no synthetic poses")
    if request["input_indices"] != [2, 3] or request["keys"] != [True, True]:
        raise ValueError("P0 fixture must contain original checkpoints P2 and P3")
    if request["yaw_explicit"] != [False, False]:
        raise ValueError("P0 fixture must contain automatic yaws")
    return payload, poses


def _pose_message(point, yaw, stamp):
    """Build one map-frame Nav2 pose using the frozen yaw."""
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = stamp
    pose.pose.position.x = float(point[0])
    pose.pose.position.y = float(point[1])
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class RealWaypointBoundaryRunner(Node):
    """Run one isolated arm while preserving request and plan provenance."""

    def __init__(self):
        super().__init__("real_waypoint_boundary_evaluation")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("arm", "current_two_key")
        self.declare_parameter("timeout_s", 95.0)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.arm = str(self.get_parameter("arm").value).strip()
        self.timeout_s = float(self.get_parameter("timeout_s").value)
        if not self.output_dir:
            raise ValueError("output_dir is required")
        if self.arm not in ARMS:
            raise ValueError(f"unknown arm: {self.arm}")
        self.fixture, self.frozen_poses = _load_fixture()
        self.odometry = []
        self.plans = []
        self.request_records = []
        self.active_request = None
        self._odom_sub = self.create_subscription(
            Odometry, "/odometry/global", self._on_odom, 20
        )
        self._plan_sub = self.create_subscription(
            NavPath, "/plan", self._on_plan, 20
        )
        self.ntp = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
        self.ntpose = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_odom(self, message):
        sample = _finite_pose(message)
        if sample is not None:
            self.odometry.append(sample)

    def _on_plan(self, message):
        points = _point_path(message)
        if not points:
            return
        record = {
            "stamp_s": _stamp(message),
            "points": points,
            "odometry_global_near_plan": self._nearest_odom(_stamp(message)),
            "request_index": None,
            "phase": "unassociated",
        }
        if self.active_request is not None:
            record["request_index"] = self.active_request["request_index"]
            record["phase"] = self.active_request["phase"]
            self.active_request["plans"].append(record)
        self.plans.append(record)

    def _nearest_odom(self, stamp_s):
        if not self.odometry:
            return None
        return min(self.odometry, key=lambda item: abs(item["stamp_s"] - stamp_s))

    def _spin_wait(self, predicate, timeout_s, label):
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not predicate():
            if time.monotonic() >= deadline:
                raise TimeoutError(label)
            rclpy.spin_once(self, timeout_sec=0.1)

    def _pose_at_dispatch(self):
        return self.odometry[-1] if self.odometry else None

    def _send_goal(self, poses, request_index, phase, *, navigate_to_pose=False):
        """Send one action and retain its exact frozen pose request."""
        dispatch_s = self.get_clock().now().nanoseconds / 1e9
        record = {
            "request_index": request_index,
            "phase": phase,
            "action_type": "NavigateToPose" if navigate_to_pose else "NavigateThroughPoses",
            "dispatch_stamp_s": dispatch_s,
            "robot_pose_at_dispatch": _pose_dict(self._pose_at_dispatch()),
            "poses": [
                {"x_m": point[0], "y_m": point[1], "yaw_rad": yaw}
                for point, yaw in poses
            ],
            "result_status": None,
            "result_stamp_s": None,
            "plans": [],
        }
        self.request_records.append(record)
        self.active_request = record
        stamp = self.get_clock().now().to_msg()
        if navigate_to_pose:
            goal = NavigateToPose.Goal()
            goal.pose = _pose_message(poses[0][0], poses[0][1], stamp)
            client = self.ntpose
            future = client.send_goal_async(goal)
        else:
            goal = NavigateThroughPoses.Goal()
            goal.poses = [_pose_message(point, yaw, stamp) for point, yaw in poses]
            client = self.ntp
            future = client.send_goal_async(goal)
        self._spin_wait(future.done, self.timeout_s, "action goal response timed out")
        handle = future.result()
        if handle is None or not handle.accepted:
            record["result_status"] = int(GoalStatus.STATUS_UNKNOWN)
            record["result_stamp_s"] = self.get_clock().now().nanoseconds / 1e9
            self.active_request = None
            return record
        result_future = handle.get_result_async()
        self._spin_wait(result_future.done, self.timeout_s, "action result timed out")
        response = result_future.result()
        record["result_status"] = int(response.status)
        record["result_stamp_s"] = self.get_clock().now().nanoseconds / 1e9
        self.active_request = None
        return record

    def _test_poses(self):
        if self.arm == "current_two_key":
            return self.frozen_poses, False
        return (self.frozen_poses[0],), self.arm == "one_pose_navigate_to_pose"

    def run(self):
        self._spin_wait(lambda: bool(self.odometry), 30.0, "global odometry unavailable")
        poses, navigate_to_pose = self._test_poses()
        client = self.ntpose if navigate_to_pose else self.ntp
        action_name = "NavigateToPose" if navigate_to_pose else "NavigateThroughPoses"
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f"{action_name} action server is unavailable")
        request = self._send_goal(
            poses, 0, ARMS[self.arm], navigate_to_pose=navigate_to_pose
        )
        reference = tuple(item[0] for item in self.frozen_poses)
        for plan in request["plans"]:
            plan["metrics"] = _plan_metrics(plan["points"], reference)
        summary = {
            "schema_version": 1,
            "conclusion": "OBSERVED",
            "classification": "UNCLASSIFIED",
            "arm": ARMS[self.arm],
            "fixture": self.fixture,
            "request": request,
            "all_plans": self.plans,
            "odometry_global": self.odometry,
            "plan_count_for_test_request": len(request["plans"]),
            "replan_count_for_test_request": max(0, len(request["plans"]) - 1),
            "planner_contract": {
                "plugin": "SmacPlannerHybrid",
                "motion_model": "DUBIN",
                "minimum_turning_radius_m": 4.0,
                "synthetic_count": 0,
            },
            "initial_state": {
                "source": "same fresh simulation spawn for both arms",
                "first_global_odometry": self.odometry[0] if self.odometry else None,
            },
        }
        manifest = {
            "schema_version": 1,
            "mode": "real_waypoint_boundary_p1",
            "arm": ARMS[self.arm],
            "fixture": str(P0_REPLAY),
            "action_type": request["action_type"],
            "topics": ["/odometry/global", "/plan"],
        }
        write_artifacts(
            self.output_dir, manifest, summary,
            {"odometry_global": self.odometry, "plans": self.plans},
        )
        return summary


def main():
    rclpy.init()
    node = None
    try:
        node = RealWaypointBoundaryRunner()
        node.run()
        return 0
    except Exception as exc:
        if node is not None:
            node.get_logger().error(str(exc))
            write_artifacts(
                node.output_dir,
                {"schema_version": 1, "mode": "real_waypoint_boundary_p1", "arm": node.arm},
                {"conclusion": "SETUP_OR_OBSERVATION_FAILURE", "errors": [str(exc)]},
                {"odometry_global": node.odometry, "plans": node.plans},
            )
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
