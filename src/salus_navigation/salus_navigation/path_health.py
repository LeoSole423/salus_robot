"""Deterministic health checks for a Nav2 path, isolated from BT plumbing."""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

import rclpy
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path
from rclpy.node import Node
from salus_interfaces.msg import PathHealth
from salus_interfaces.srv import EvaluatePathHealth
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass(frozen=True)
class CostmapView:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    stamp_s: float
    # Humble represents Costmap.data as array.array('B'); retain it instead of
    # materializing a Python tuple for the whole rolling grid on every update.
    data: Sequence[int]


@dataclass(frozen=True)
class HealthResult:
    state: int
    reason: str
    costmap_age_s: float
    max_cost: int
    checked_samples: int
    cross_track_error_m: float


def path_signature(path: Path) -> tuple[object, ...]:
    poses = path.poses
    if not poses:
        return (path.header.frame_id, 0)
    return (path.header.frame_id, len(poses), tuple((round(item.pose.position.x, 2), round(item.pose.position.y, 2)) for item in (poses[0], poses[len(poses) // 2], poses[-1])))


def point_to_segment_distance(x: float, y: float, ax: float, ay: float, bx: float, by: float) -> tuple[float, float]:
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-9:
        return math.hypot(x - ax, y - ay), 0.0
    ratio = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / length_sq))
    return math.hypot(x - (ax + ratio * dx), y - (ay + ratio * dy)), ratio


def cross_track_error(path: Path, x: float, y: float) -> tuple[float, float]:
    if len(path.poses) < 2:
        return 0.0, 0.0
    best_distance, progress, walked = float("inf"), 0.0, 0.0
    for left, right in zip(path.poses, path.poses[1:]):
        ax, ay = left.pose.position.x, left.pose.position.y
        bx, by = right.pose.position.x, right.pose.position.y
        length = math.hypot(bx - ax, by - ay)
        distance, ratio = point_to_segment_distance(x, y, ax, ay, bx, by)
        if distance < best_distance:
            best_distance, progress = distance, walked + ratio * length
        walked += length
    return best_distance, progress


class PathHealthPolicy:
    """Pure policy: geometry, hysteresis and progress; no ROS side effects."""

    def __init__(self, *, max_distance_m=12.0, sample_step_m=0.25, high_cost=100, lethal_cost=253, high_samples=3, cross_track_replan_m=0.9, cross_track_recover_m=0.6, cross_track_confirmations=3, cooldown_s=1.5, costmap_timeout_s=1.5, progress_timeout_s=5.0) -> None:
        self.max_distance_m, self.sample_step_m = max_distance_m, sample_step_m
        self.high_cost, self.lethal_cost, self.high_samples = high_cost, lethal_cost, high_samples
        self.cross_track_replan_m, self.cross_track_recover_m = cross_track_replan_m, cross_track_recover_m
        self.cross_track_confirmations, self.cooldown_s = cross_track_confirmations, cooldown_s
        self.costmap_timeout_s, self.progress_timeout_s = costmap_timeout_s, progress_timeout_s
        self._cross_count = 0; self._last_replan_s = -float("inf")
        self._signature = None; self._best_progress = 0.0; self._progress_at_s = 0.0

    @staticmethod
    def _cell_cost(costmap: CostmapView, x: float, y: float) -> int:
        col = math.floor((x - costmap.origin_x) / costmap.resolution)
        row = math.floor((y - costmap.origin_y) / costmap.resolution)
        if not (0 <= col < costmap.width and 0 <= row < costmap.height):
            return 0
        return int(costmap.data[row * costmap.width + col])

    def _footprint_cost(self, costmap: CostmapView, x: float, y: float, yaw: float) -> int:
        # Canonical footprint corners plus centre: sufficient conservative
        # raster sampling for a 0.25 m global costmap without hidden geometry.
        corners = ((0.0, 0.0), (1.05, .38), (1.05, -.38), (-.12, -.38), (-.12, .38))
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        return max(self._cell_cost(costmap, x + dx * cos_yaw - dy * sin_yaw, y + dx * sin_yaw + dy * cos_yaw) for dx, dy in corners)

    def _result(self, state, reason, age, maximum, checked, error):
        return HealthResult(state, reason, age, maximum, checked, error)

    def evaluate(self, path: Path, *, robot_x: float, robot_y: float,
                 costmap: CostmapView | None, now_s: float,
                 tf_available=True, track_active_state=True) -> HealthResult:
        if not tf_available:
            return self._result(PathHealth.STOP_AND_WAIT, "tf_unavailable", float("inf"), 0, 0, 0.0)
        if costmap is None:
            return self._result(PathHealth.STOP_AND_WAIT, "costmap_unavailable", float("inf"), 0, 0, 0.0)
        age = max(0.0, now_s - costmap.stamp_s)
        if age > self.costmap_timeout_s:
            return self._result(PathHealth.STOP_AND_WAIT, "costmap_stale", age, 0, 0, 0.0)
        if len(path.poses) < 2:
            return self._result(PathHealth.REPLAN, "path_too_short", age, 0, 0, 0.0)

        signature = path_signature(path)
        error, progress = cross_track_error(path, robot_x, robot_y)
        if not track_active_state:
            # A candidate is evaluated independently.  Its validity must not
            # reset the progress, hysteresis or cooldown of the active path.
            return self._evaluate_geometry(path, progress, error, costmap, now_s, age)
        if signature != self._signature:
            self._signature, self._best_progress, self._progress_at_s, self._cross_count = signature, progress, now_s, 0
        elif progress > self._best_progress + 0.05:
            self._best_progress, self._progress_at_s = progress, now_s

        if error >= self.cross_track_replan_m:
            self._cross_count += 1
        elif error <= self.cross_track_recover_m:
            self._cross_count = 0

        geometry = self._evaluate_geometry(path, progress, error, costmap, now_s, age)
        if geometry.state == PathHealth.REPLAN:
            self._last_replan_s = now_s
            return geometry

        if now_s - self._last_replan_s >= self.cooldown_s:
            if self._cross_count >= self.cross_track_confirmations:
                self._last_replan_s = now_s
                return self._result(PathHealth.REPLAN, "cross_track_error", geometry.costmap_age_s,
                                    geometry.max_cost, geometry.checked_samples, error)
            if now_s - self._progress_at_s >= self.progress_timeout_s:
                self._last_replan_s = now_s
                return self._result(PathHealth.REPLAN, "progress_stalled", geometry.costmap_age_s,
                                    geometry.max_cost, geometry.checked_samples, error)
        return geometry

    def _evaluate_geometry(self, path: Path, progress: float, error: float,
                           costmap: CostmapView, now_s: float, age: float) -> HealthResult:
        # Only inspect the part the vehicle still has to traverse.  Checking
        # the path behind the rear axle would cause needless replans when a
        # new obstacle appears after the robot has already passed it.
        maximum = checked = sustained = 0
        remaining = self.max_distance_m
        walked = 0.0
        for left, right in zip(path.poses, path.poses[1:]):
            ax, ay = left.pose.position.x, left.pose.position.y; bx, by = right.pose.position.x, right.pose.position.y
            dx, dy = bx - ax, by - ay; length = math.hypot(dx, dy)
            if length <= 1.0e-6: continue
            segment_start = max(0.0, progress - walked)
            yaw, distance = math.atan2(dy, dx), segment_start
            while distance <= length and remaining >= 0.0:
                ratio = distance / length
                cost = self._footprint_cost(costmap, ax + ratio * dx, ay + ratio * dy, yaw)
                maximum, checked = max(maximum, cost), checked + 1
                if cost >= self.lethal_cost:
                    return self._result(PathHealth.REPLAN, "path_collision", age, maximum, checked, error)
                sustained = sustained + 1 if cost >= self.high_cost else 0
                if sustained >= self.high_samples:
                    return self._result(PathHealth.REPLAN, "clearance_degraded", age, maximum, checked, error)
                distance += self.sample_step_m; remaining -= self.sample_step_m
            walked += length
            if remaining < 0.0:
                break

        return self._result(PathHealth.KEEP_PATH, "path_healthy", age, maximum, checked, error)


def costmap_view_from_message(message: Costmap) -> CostmapView:
    """Adapt a received Costmap without copying its cell sequence."""
    meta = message.metadata
    stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
    return CostmapView(
        meta.resolution, meta.size_x, meta.size_y,
        meta.origin.position.x, meta.origin.position.y, stamp, message.data,
    )


class PathHealthNode(Node):
    """ROS boundary around PathHealthPolicy, consumed by the thin BT plugin."""

    def __init__(self) -> None:
        super().__init__("path_health")
        for name, value in {
            "costmap_topic": "/global_costmap/costmap_raw",
            "service_name": "/path_health/evaluate",
            "health_topic": "/path_health",
            "base_frame": "base_footprint",
            "costmap_timeout_s": 1.5,
            "tf_timeout_s": 1.5,
        }.items():
            self.declare_parameter(name, value)
        self._costmap = None
        self._policy = PathHealthPolicy(costmap_timeout_s=float(self.get_parameter("costmap_timeout_s").value))
        self._pub = self.create_publisher(PathHealth, str(self.get_parameter("health_topic").value), 10)
        self.create_subscription(Costmap, str(self.get_parameter("costmap_topic").value), self._on_costmap, 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.create_service(EvaluatePathHealth, str(self.get_parameter("service_name").value), self._on_evaluate)

    def _on_costmap(self, message):
        self._costmap_frame = message.header.frame_id
        self._costmap = costmap_view_from_message(message)

    def _on_evaluate(self, request, response):
        now_s = self.get_clock().now().nanoseconds * 1.0e-9
        path_frame = request.path.header.frame_id
        if len(request.path.poses) < 2:
            # There is no active route on the first tick.  This is the one
            # deliberate reason to enter the planner branch without TF.
            result = self._policy._result(PathHealth.REPLAN, "path_missing", 0.0, 0, 0, 0.0)
        elif not path_frame:
            result = self._policy._result(PathHealth.STOP_AND_WAIT, "path_frame_missing", float("inf"), 0, 0, 0.0)
        elif self._costmap is not None and path_frame != self._costmap_frame:
            result = self._policy._result(PathHealth.STOP_AND_WAIT, "costmap_frame_mismatch", float("inf"), 0, 0, 0.0)
        else:
            try:
                # Use the BufferCore query directly: it is non-blocking.  A
                # service callback must never wait for TF subscription work on
                # the same executor, otherwise Nav2 can time out its BT tick.
                transform = self._tf_buffer.lookup_transform_core(
                    path_frame, str(self.get_parameter("base_frame").value), rclpy.time.Time())
                stamp = transform.header.stamp
                age = max(0.0, now_s - stamp.sec - stamp.nanosec * 1.0e-9)
                if age > float(self.get_parameter("tf_timeout_s").value):
                    raise TransformException("transform_stale")
                point = transform.transform.translation
                result = self._policy.evaluate(
                    request.path, robot_x=point.x, robot_y=point.y,
                    costmap=self._costmap, now_s=now_s,
                    track_active_state=request.context == EvaluatePathHealth.Request.ACTIVE)
                if request.context == EvaluatePathHealth.Request.CANDIDATE and result.state != PathHealth.KEEP_PATH:
                    result = self._policy._result(
                        PathHealth.STOP_AND_WAIT,
                        f"candidate_invalid:{result.reason}",
                        result.costmap_age_s, result.max_cost,
                        result.checked_samples, result.cross_track_error_m)
            except Exception as exc:
                # The BT service has a bounded timeout.  A malformed or
                # temporarily unavailable TF lookup must become a safe result,
                # never an unhandled callback that leaves Nav2 waiting.
                result = self._policy._result(
                    PathHealth.STOP_AND_WAIT,
                    f"tf_unavailable:{type(exc).__name__}",
                    float("inf"), 0, 0, 0.0)
        message = PathHealth(); message.stamp = self.get_clock().now().to_msg(); message.state, message.reason = result.state, result.reason
        message.costmap_age_s, message.max_cost, message.checked_samples, message.cross_track_error_m = result.costmap_age_s, result.max_cost, result.checked_samples, result.cross_track_error_m
        self._pub.publish(message); response.health = message
        return response


def main(args=None):
    rclpy.init(args=args); node = PathHealthNode()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
