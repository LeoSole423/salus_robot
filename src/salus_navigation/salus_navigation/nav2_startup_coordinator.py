"""Activate Nav2 only after simulation inputs and TF are causally ready."""

from __future__ import annotations

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from salus_interfaces.srv import EvaluatePathHealth
from tf2_ros import Buffer, TransformException, TransformListener

from .startup_readiness import ReadinessSnapshot, StartupPolicy, StartupState


NAV2_NODES = (
    "planner_server", "controller_server", "smoother_server",
    "bt_navigator", "behavior_server",
)


def _stamp_ns(message) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _finite_odometry(message: Odometry) -> bool:
    pose, twist = message.pose.pose, message.twist.twist
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z,
    )
    norm = sum(value * value for value in (
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    ))
    return all(math.isfinite(value) for value in values) and norm > 1.0e-12


def _valid_scan(message: LaserScan) -> bool:
    return (
        message.header.frame_id == "base_footprint"
        and len(message.ranges) > 0
        and all(math.isfinite(value) for value in (
            message.angle_min, message.angle_max, message.angle_increment,
            message.range_min, message.range_max,
        ))
        and message.angle_increment > 0.0
        and message.range_max > message.range_min >= 0.0
    )


class Nav2StartupCoordinator(Node):
    """ROS adapter around the pure startup policy."""

    def __init__(self) -> None:
        super().__init__("nav2_startup_coordinator")
        self.declare_parameter("use_keepout", True)
        self.declare_parameter("obstacle_detection_required", True)
        self.declare_parameter("input_freshness_s", 2.0)
        self.declare_parameter("activation_timeout_s", 30.0)
        self._use_keepout = bool(self.get_parameter("use_keepout").value)
        self._obstacle_detection_required = bool(
            self.get_parameter("obstacle_detection_required").value
        )
        self._freshness_s = float(self.get_parameter("input_freshness_s").value)
        self._activation_timeout_s = float(self.get_parameter("activation_timeout_s").value)
        self._policy = StartupPolicy()
        self._clock_samples: list[int] = []
        self._odom_samples: list[Odometry] = []
        self._scan: LaserScan | None = None
        self._scan_received_at = 0.0
        self._mask_ready = not self._use_keepout
        self._manage_future = None
        self._manage_requested_at = 0.0
        self._state_futures = {name: None for name in NAV2_NODES}
        self._state_requested_at = {name: 0.0 for name in NAV2_NODES}
        self._node_states = {name: "unknown" for name in NAV2_NODES}
        self._path_health_future = None
        self._path_health_requested_at = 0.0
        self._path_health_ready = False
        self._tf_available = False
        self._tf_fresh = False
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, self)
        self._manage = self.create_client(
            ManageLifecycleNodes, "/lifecycle_manager_navigation/manage_nodes"
        )
        self._path_health = self.create_client(
            EvaluatePathHealth, "/path_health/evaluate"
        )
        self._state_clients = {
            name: self.create_client(GetState, f"/{name}/get_state") for name in NAV2_NODES
        }
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(Odometry, "/odometry/global", self._on_odom, 10)
        self.create_subscription(
            LaserScan, "/scan_clean", self._on_scan, qos_profile_sensor_data
        )
        mask_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, "/keepout_filter_mask", self._on_mask, mask_qos
        )
        self._diagnostics = self.create_publisher(
            DiagnosticArray, "/navigation_startup/diagnostics", 10
        )
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_diagnostics)

    def _on_clock(self, message: Clock) -> None:
        value = int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)
        if not self._clock_samples or value != self._clock_samples[-1]:
            self._clock_samples = (self._clock_samples + [value])[-2:]

    def _on_odom(self, message: Odometry) -> None:
        self._odom_samples = (self._odom_samples + [message])[-2:]

    def _on_scan(self, message: LaserScan) -> None:
        self._scan = message
        self._scan_received_at = time.monotonic()

    def _on_mask(self, message: OccupancyGrid) -> None:
        self._mask_ready = (
            message.header.frame_id == "map"
            and message.info.width > 0 and message.info.height > 0
            and len(message.data) == message.info.width * message.info.height
        )

    def _update_tf(self) -> None:
        self._tf_available = False
        self._tf_fresh = False
        try:
            transform = self._tf.lookup_transform("map", "base_footprint", Time())
            self._tf_available = True
            if self._clock_samples:
                transform_ns = (
                    int(transform.header.stamp.sec) * 1_000_000_000
                    + int(transform.header.stamp.nanosec)
                )
                age_s = (self._clock_samples[-1] - transform_ns) / 1.0e9
                self._tf_fresh = -0.1 <= age_s <= self._freshness_s
        except TransformException:
            pass

    def _snapshot(self) -> ReadinessSnapshot:
        clock_progressive = (
            len(self._clock_samples) >= 2 and self._clock_samples[-1] > self._clock_samples[-2]
        )
        odometry_progressive = (
            len(self._odom_samples) >= 2
            and _stamp_ns(self._odom_samples[-1]) > _stamp_ns(self._odom_samples[-2])
        )
        scan_fresh = (
            self._scan is not None
            and time.monotonic() - self._scan_received_at <= self._freshness_s
        )
        return ReadinessSnapshot(
            clock_progressive=clock_progressive,
            odometry_progressive=odometry_progressive,
            odometry_finite=bool(self._odom_samples) and _finite_odometry(self._odom_samples[-1]),
            transform_available=self._tf_available,
            transform_fresh=self._tf_fresh,
            scan_valid=self._scan is not None and _valid_scan(self._scan),
            scan_fresh=scan_fresh,
            obstacle_detection_required=self._obstacle_detection_required,
            keepout_required=self._use_keepout,
            keepout_ready=self._mask_ready,
            lifecycle_manager_ready=self._manage.service_is_ready(),
        )

    def _request_startup(self) -> None:
        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        self._manage_future = self._manage.call_async(request)
        self._manage_requested_at = time.monotonic()

    def _poll_path_health(self, now: float) -> bool:
        """Require one real service round-trip before advertising Nav2 ready."""
        if self._path_health_ready:
            return True
        if self._path_health_future is not None:
            if self._path_health_future.done():
                try:
                    self._path_health_ready = self._path_health_future.result() is not None
                except Exception:
                    self._path_health_ready = False
                self._path_health_future = None
                return self._path_health_ready
            if now - self._path_health_requested_at > 2.0:
                self._path_health_future.cancel()
                self._path_health_future = None
            return False
        if self._path_health.service_is_ready():
            request = EvaluatePathHealth.Request()
            request.path = Path()
            request.context = EvaluatePathHealth.Request.ACTIVE
            self._path_health_future = self._path_health.call_async(request)
            self._path_health_requested_at = now
        return False

    def _poll_startup(self) -> None:
        now = time.monotonic()
        if self._manage_future is not None:
            if self._manage_future.done():
                response = self._manage_future.result()
                self._manage_future = None
                if response is None or not response.success:
                    self._policy.activation_failed("LIFECYCLE_START_REJECTED")
                    return
            elif now - self._manage_requested_at > self._activation_timeout_s:
                self._manage_future.cancel()
                self._policy.activation_failed("LIFECYCLE_START_TIMEOUT")
                return
        for name, client in self._state_clients.items():
            future = self._state_futures[name]
            if future is not None:
                if future.done():
                    response = future.result()
                    self._node_states[name] = (
                        response.current_state.label if response is not None else "invalid"
                    )
                    self._state_futures[name] = None
                elif now - self._state_requested_at[name] > 2.0:
                    future.cancel()
                    self._state_futures[name] = None
            if self._state_futures[name] is None and client.service_is_ready():
                self._state_futures[name] = client.call_async(GetState.Request())
                self._state_requested_at[name] = now
        if (all(state == "active" for state in self._node_states.values())
                and self._poll_path_health(now)):
            self._policy.activation_succeeded()
        elif now - self._manage_requested_at > self._activation_timeout_s:
            self._policy.activation_failed("NAV2_NODES_NOT_ACTIVE")

    def _tick(self) -> None:
        if self._policy.state is StartupState.WAITING_INPUTS:
            self._update_tf()
            if self._policy.observe(self._snapshot()):
                self._request_startup()
        elif self._policy.state is StartupState.STARTING:
            self._poll_startup()

    def _publish_diagnostics(self) -> None:
        snapshot = self._snapshot()
        status = DiagnosticStatus()
        status.name = "navigation_startup"
        status.hardware_id = "salus_navigation"
        # diagnostic_msgs/DiagnosticStatus uses a byte field in Humble; its
        # generated constants already have the required one-byte type.
        status.level = (
            DiagnosticStatus.OK if self._policy.state is StartupState.ACTIVE
            else DiagnosticStatus.ERROR if self._policy.state is StartupState.FAILED
            else DiagnosticStatus.WARN
        )
        status.message = f"{self._policy.state.value}: {self._policy.reason}"
        values = {**snapshot.__dict__, "state": self._policy.state.value,
                  "reason": self._policy.reason,
                  "path_health_preflight": self._path_health_ready,
                  **self._node_states}
        status.values = [KeyValue(key=str(key), value=str(value)) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._diagnostics.publish(message)


def main(args=None) -> None:
    """Run the startup coordinator."""
    rclpy.init(args=args)
    node = Nav2StartupCoordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
