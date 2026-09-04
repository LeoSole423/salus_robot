"""PC-only runtime coverage for the real Nav2/safety composition.

The fixture supplies the external sensor and TF contracts.  It deliberately
does not publish ``/clock`` and does not provide a hardware or actuation node.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

# This must happen before importing or initializing rclpy.
_DOMAIN_ID = os.environ.get(
    "SALUS_NAVIGATION_REAL_DOMAIN_ID",
    str(70 + (os.getpid() % 100)),
)
os.environ["ROS_DOMAIN_ID"] = _DOMAIN_ID

import rclpy  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Point, TransformStamped, Twist  # noqa: E402
from lifecycle_msgs.msg import State  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from nav2_msgs.action import ComputePathToPose  # noqa: E402
from nav2_msgs.msg import CollisionMonitorState, Costmap  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from robot_localization.srv import FromLL  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from salus_interfaces.msg import CmdVelFinal, ProjectedKeepoutState  # noqa: E402
from salus_interfaces.srv import GetZonesState, SetZonesGeoJson  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402


PACKAGE_LAUNCH = ["ros2", "launch", "salus_navigation"]


def _spin_until(node: Node, predicate, timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError(f"timeout waiting for {description}")


def _call(node: Node, client, request, timeout_s: float = 15.0):
    _spin_until(node, client.service_is_ready, timeout_s, "service discovery")
    future = client.call_async(request)
    _spin_until(node, future.done, timeout_s, "service response")
    response = future.result()
    assert response is not None
    return response


def _stop_launch(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=10.0)


class SyntheticNavigationInputs(Node):
    """External PC fixture for odometry, TF, clean scan and command input."""

    def __init__(self) -> None:
        super().__init__("navigation_real_pc_fixture")
        self.obstacle_range: float | None = None
        self.scan_enabled = True
        self.command_enabled = False
        self.safe: list[Twist] = []
        self.final: list[CmdVelFinal] = []
        self.final_times: list[float] = []
        self.startup_values: dict[str, str] = {}
        self.local_costmaps: list[Costmap] = []
        self.global_costmaps: list[Costmap] = []
        self.monitor_states: list[CollisionMonitorState] = []
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan_clean", qos_profile_sensor_data
        )
        self.local_odom_pub = self.create_publisher(Odometry, "/odometry/local", 10)
        self.global_odom_pub = self.create_publisher(Odometry, "/odometry/global", 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self.safe.append, 10)
        self.create_subscription(
            CmdVelFinal, "/cmd_vel_final", self._on_final, 10
        )
        self.create_subscription(
            DiagnosticArray, "/navigation_startup/diagnostics", self._on_startup, 10
        )
        self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", self.local_costmaps.append, 10
        )
        self.create_subscription(
            Costmap, "/global_costmap/costmap_raw", self.global_costmaps.append, 10
        )
        self.create_subscription(
            CollisionMonitorState,
            "/collision_monitor_state",
            self.monitor_states.append,
            10,
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self._timer = self.create_timer(0.05, self._publish_inputs)

    def _on_final(self, message: CmdVelFinal) -> None:
        self.final.append(message)
        self.final_times.append(time.monotonic())

    def _on_startup(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "navigation_startup":
                self.startup_values = {
                    item.key: item.value for item in status.values
                }

    def _publish_inputs(self) -> None:
        stamp = self.get_clock().now().to_msg()
        local = Odometry()
        local.header.stamp = stamp
        local.header.frame_id = "odom"
        local.child_frame_id = "base_footprint"
        local.pose.pose.orientation.w = 1.0
        local.twist.twist.linear.x = 0.0
        global_odom = Odometry()
        global_odom.header.stamp = stamp
        global_odom.header.frame_id = "map"
        global_odom.child_frame_id = "base_footprint"
        global_odom.pose.pose.orientation.w = 1.0
        self.local_odom_pub.publish(local)
        self.global_odom_pub.publish(global_odom)

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_footprint"
        odom_to_base.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform([map_to_odom, odom_to_base])

        if self.scan_enabled:
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = "base_footprint"
            scan.angle_min = -math.pi / 2.0
            scan.angle_max = math.pi / 2.0
            scan.angle_increment = math.pi / 359.0
            scan.scan_time = 0.05
            scan.range_min = 0.4
            scan.range_max = 20.0
            scan.ranges = [float("inf")] * 360
            if self.obstacle_range is not None:
                for index in range(176, 185):
                    scan.ranges[index] = self.obstacle_range
            self.scan_pub.publish(scan)
        if self.command_enabled:
            command = Twist()
            command.linear.x = 1.0
            self.cmd_pub.publish(command)


def _launch(command: list[str], log_path: Path) -> subprocess.Popen:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = _DOMAIN_ID
    environment["RCUTILS_COLORIZED_OUTPUT"] = "0"
    log_file = log_path.open("w", encoding="utf-8")
    return subprocess.Popen(
        command,
        env=environment,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _assert_lifecycle_active(node: Node, node_name: str) -> None:
    client = node.create_client(GetState, f"/{node_name}/get_state")
    _spin_until(node, client.service_is_ready, 20.0, f"{node_name} state service")
    future = client.call_async(GetState.Request())
    _spin_until(node, future.done, 10.0, f"{node_name} state response")
    response = future.result()
    assert response is not None
    assert response.current_state.id == State.PRIMARY_STATE_ACTIVE, (
        node_name,
        response.current_state.label,
    )


def _run_navigation_real_runtime(log_path: Path, runtime_dir: Path) -> None:
    rclpy.init(domain_id=int(_DOMAIN_ID))
    fixture = SyntheticNavigationInputs()
    launch_process = _launch(
        PACKAGE_LAUNCH
        + [
            "navigation_real.launch.py",
            "use_keepout:=true",
            f"zones_runtime_dir:={runtime_dir}",
        ],
        log_path,
    )
    try:
        _spin_until(
            fixture,
            lambda: fixture.startup_values.get("projected_keepouts_ready") == "True",
            30.0,
            "zones_manager mask_ready",
        )
        zones = fixture.create_client(GetZonesState, "/zones_manager/get_state")
        zones_state = _call(fixture, zones, GetZonesState.Request())
        assert zones_state.ok and zones_state.mask_ready
        assert zones_state.frame_id == "map"

        try:
            _spin_until(
                fixture,
                lambda: fixture.startup_values.get("state") == "ACTIVE",
                45.0,
                "navigation_startup ACTIVE without /clock",
            )
        except AssertionError as exc:
            raise AssertionError(
                f"{exc}; readiness={fixture.startup_values}; "
                f"launch_log={log_path.read_text(encoding='utf-8')[-12000:]}"
            ) from exc
        for node_name in (
            "planner_server", "controller_server", "bt_navigator", "behavior_server",
        ):
            _assert_lifecycle_active(fixture, node_name)
        _spin_until(
            fixture,
            lambda: bool(fixture.local_costmaps) and bool(fixture.global_costmaps),
            20.0,
            "local and global costmaps",
        )
        assert fixture.count_publishers("/clock") == 0
        assert fixture.count_publishers("/cmd_vel_safe") == 1
        assert fixture.count_publishers("/cmd_vel_final") == 1

        planner = ActionClient(fixture, ComputePathToPose, "/compute_path_to_pose")
        _spin_until(fixture, planner.server_is_ready, 20.0, "ComputePathToPose action")
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.planner_id = "GridBased"
        for pose, x in ((goal.start, 0.0), (goal.goal, 8.0)):
            pose.header.stamp = fixture.get_clock().now().to_msg()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
        send_future = planner.send_goal_async(goal)
        _spin_until(fixture, send_future.done, 15.0, "ComputePathToPose goal response")
        handle = send_future.result()
        assert handle is not None and handle.accepted
        result_future = handle.get_result_async()
        _spin_until(fixture, result_future.done, 20.0, "ComputePathToPose result")
        result = result_future.result().result
        assert result is not None
        if hasattr(result, "error_code"):
            assert result.error_code == ComputePathToPose.Result.NONE, getattr(
                result, "error_msg", "planner returned an error"
            )
        assert result.path.header.frame_id == "map"
        assert result.path.poses
        for pose in result.path.poses:
            assert all(
                math.isfinite(value)
                for value in (
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                )
            )

        fixture.command_enabled = True
        fixture.final.clear()
        fixture.final_times.clear()
        _spin_until(
            fixture,
            lambda: any(
                message.source == CmdVelFinal.SOURCE_AUTO
                and message.twist.linear.x > 0.5
                for message in fixture.final
            ),
            10.0,
            "forward command through /cmd_vel_final",
        )

        # Let the safety owner observe the cluster before asserting the final
        # command boundary, avoiding an in-flight clear-scan command.
        fixture.obstacle_range = 0.5
        fixture.safe.clear()
        _spin_until(
            fixture,
            lambda: any(
                message.linear.x == 0.0 and message.angular.z == 0.0
                for message in fixture.safe
            ),
            5.0,
            "collision monitor safe stop for footprint obstacle",
        )
        fixture.final.clear()
        fixture.final_times.clear()
        obstacle_started = time.monotonic()
        _spin_until(
            fixture,
            lambda: any(
                message.twist.linear.x == 0.0
                and message.twist.angular.z == 0.0
                for message in fixture.final
            ),
            10.0,
            "safe stop for obstacle inside footprint",
        )
        assert not any(
            message.source == CmdVelFinal.SOURCE_AUTO
            and message.twist.linear.x > 0.0
            and timestamp >= obstacle_started
            for message, timestamp in zip(fixture.final, fixture.final_times[-len(fixture.final):])
        )

        fixture.final.clear()
        fixture.final_times.clear()
        fixture.obstacle_range = None
        fixture.scan_enabled = False
        stale_started = time.monotonic()
        timeout_boundary = stale_started + 1.1
        _spin_until(
            fixture,
            lambda: time.monotonic() - stale_started > 1.8,
            3.0,
            "source timeout after stopping /scan_clean",
        )
        assert not any(
            message.twist.linear.x > 0.0
            for message, timestamp in zip(fixture.final, fixture.final_times)
            if message.source == CmdVelFinal.SOURCE_AUTO and timestamp >= timeout_boundary
        )
        assert launch_process.poll() is None
    finally:
        _stop_launch(launch_process)
        fixture.destroy_node()
        rclpy.shutdown()


def test_navigation_real_pc_runtime_without_clock() -> None:
    """Exercise startup, planning and safety with only synthetic PC inputs."""
    with tempfile.TemporaryDirectory(prefix="salus-navigation-real-", dir="/tmp") as temp:
        root = Path(temp)
        _run_navigation_real_runtime(root / "navigation_real.log", root / "empty-zones")


class ZonesFromLLFixture(Node):
    """Minimal local datum converter for the isolated zones contract test."""

    def __init__(self) -> None:
        super().__init__("zones_real_fromll_fixture")
        self.create_service(FromLL, "/fromLL", self._from_ll)
        projected_qos = QoSProfile(depth=1)
        projected_qos.reliability = ReliabilityPolicy.RELIABLE
        projected_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.projected: list[ProjectedKeepoutState] = []
        self.create_subscription(
            ProjectedKeepoutState,
            "/zones_manager/projected_keepouts",
            self.projected.append,
            projected_qos,
        )

    @staticmethod
    def _from_ll(request: FromLL.Request, response: FromLL.Response):
        response.map_point = Point()
        response.map_point.x = request.ll_point.longitude
        response.map_point.y = request.ll_point.latitude
        response.map_point.z = request.ll_point.altitude
        return response


def test_navigation_zones_real_projects_one_small_polygon() -> None:
    """Keepout projection is covered independently from the Nav2 long-range smoke."""
    rclpy.init(domain_id=int(_DOMAIN_ID))
    fixture = ZonesFromLLFixture()
    with tempfile.TemporaryDirectory(prefix="salus-zones-real-", dir="/tmp") as temp:
        root = Path(temp)
        process = _launch(
            PACKAGE_LAUNCH
            + [
                "navigation_zones_real.launch.py",
                "use_keepout:=true",
                f"runtime_dir:={root / 'empty-zones'}",
            ],
            root / "zones_real.log",
        )
        try:
            setter = fixture.create_client(SetZonesGeoJson, "/zones_manager/set_geojson")
            getter = fixture.create_client(GetZonesState, "/zones_manager/get_state")
            _spin_until(fixture, setter.service_is_ready, 20.0, "SetZonesGeoJson service")
            _spin_until(fixture, getter.service_is_ready, 20.0, "GetZonesState service")
            document = {
                "type": "FeatureCollection",
                "features": [{
                    "type": "Feature",
                    "properties": {"id": "small", "enabled": True},
                    "geometry": {
                        "type": "Polygon",
                        "coordinates": [[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 0.0]]],
                    },
                }],
            }
            response = _call(
                fixture,
                setter,
                SetZonesGeoJson.Request(geojson=json.dumps(document)),
            )
            assert response.ok
            _spin_until(
                fixture,
                lambda: bool(fixture.projected)
                and fixture.projected[-1].header.frame_id == "map"
                and len(fixture.projected[-1].polygons) == 1,
                10.0,
                "one projected map-frame polygon",
            )
            state = _call(fixture, getter, GetZonesState.Request())
            assert state.ok and state.mask_ready
            assert state.frame_id == "map"
            assert len(fixture.projected[-1].polygons) == 1
        finally:
            _stop_launch(process)
            fixture.destroy_node()
            rclpy.shutdown()
