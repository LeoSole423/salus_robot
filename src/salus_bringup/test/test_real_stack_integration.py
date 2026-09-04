"""PC-only integration gate for the already-scoped real subsystem profiles."""

from __future__ import annotations

import math
import os
import signal
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import pytest


_DOMAIN_ID = os.environ.get(
    "SALUS_REAL_STACK_INTEGRATION_DOMAIN_ID",
    str(20 + (os.getpid() % 180)),
)

rclpy = pytest.importorskip("rclpy")
from diagnostic_msgs.msg import DiagnosticArray  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from lifecycle_msgs.msg import State  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from nav2_msgs.action import ComputePathToPose  # noqa: E402
from nav2_msgs.msg import Costmap  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from salus_interfaces.msg import (  # noqa: E402
    CmdVelFinal,
    DriveTelemetry,
    GnssRtkStatus,
)
from salus_interfaces.srv import GetZonesState  # noqa: E402
from robot_localization.srv import FromLL  # noqa: E402
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus, PointCloud2  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
from tf2_ros import Buffer, TransformException, TransformListener  # noqa: E402


DATUM_LAT = -31.4859026607927
DATUM_LON = -64.24097358249034
LAUNCHES = (
    ("description", ("ros2", "launch", "salus_description", "description_real.launch.py")),
    (
        "localization_local",
        ("ros2", "launch", "salus_localization", "localization_local_real.launch.py"),
    ),
    (
        "localization_global",
        ("ros2", "launch", "salus_localization", "global_localization_real.launch.py"),
    ),
    ("perception", ("ros2", "launch", "salus_perception", "perception_real.launch.py")),
)
NAVIGATION_LAUNCH = (
    "navigation",
    ("ros2", "launch", "salus_navigation", "navigation_real.launch.py"),
)
EXPECTED_NODES = {
    "robot_state_publisher",
    "ackermann_odometry",
    "salus_local_ekf",
    "global_stationary_gates",
    "gps_course_heading",
    "orientation_source_selector",
    "navsat_transform",
    "salus_global_ekf",
    "scan_ground_filter",
    "pointcloud_to_laserscan",
    "scan_noise_filter",
    "zones_manager",
    "collision_monitor",
    "lifecycle_manager_collision_monitor_real",
    "planner_server",
    "controller_server",
    "bt_navigator",
    "behavior_server",
    "nav_observer",
    "path_health",
    "navigation_profile_coordinator",
    "lifecycle_manager_navigation",
    "nav2_startup_coordinator",
    "nav_command_server",
}


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _finite_odometry(message: Odometry) -> bool:
    values = (
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        message.pose.pose.position.z,
        message.pose.pose.orientation.x,
        message.pose.pose.orientation.y,
        message.pose.pose.orientation.z,
        message.pose.pose.orientation.w,
        message.twist.twist.linear.x,
        message.twist.twist.angular.z,
    )
    return all(math.isfinite(value) for value in values)


def _progressive(messages: list, minimum: int = 3) -> bool:
    if len(messages) < minimum:
        return False
    stamps = [_stamp_seconds(message.header.stamp) for message in messages[-minimum:]]
    return all(math.isfinite(value) for value in stamps) and all(
        newer > older for older, newer in zip(stamps, stamps[1:])
    )


def _stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=10.0)


class RealStackIntegrationHarness(Node):
    """Publish only physical input boundaries and observe product outputs."""

    def __init__(self) -> None:
        super().__init__("real_stack_integration_fixture")
        self._start_ns = self.get_clock().now().nanoseconds
        self.cloud_enabled = True
        self.cloud_mode = "clear"
        self.command_enabled = False
        self.wheel_samples: list[Odometry] = []
        self.local_samples: list[Odometry] = []
        self.global_samples: list[Odometry] = []
        self.heading_samples: list[Imu] = []
        self.orientation_samples: list[Imu] = []
        self.gps_samples: list[Odometry] = []
        self.obstacle_clouds: list[PointCloud2] = []
        self.scans: list[LaserScan] = []
        self.clean_scans: list[tuple[LaserScan, float]] = []
        self.final_commands: list[tuple[CmdVelFinal, float]] = []
        self.safe_commands: list[tuple[Twist, float]] = []
        self.startup_values: dict[str, str] = {}
        self.local_costmaps: list[Costmap] = []
        self.global_costmaps: list[Costmap] = []

        self.drive_pub = self.create_publisher(
            DriveTelemetry, "/controller/drive_telemetry", 10
        )
        self.imu_pub = self.create_publisher(Imu, "/salus/imu/data", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/salus/gps/fix", 10)
        self.rtk_pub = self.create_publisher(
            GnssRtkStatus,
            "/salus/hardware/gnss_primary/rtk_status",
            10,
        )
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/scan_3d", qos_profile_sensor_data
        )
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(Odometry, "/wheel/odometry", self.wheel_samples.append, 50)
        self.create_subscription(Odometry, "/odometry/local", self.local_samples.append, 50)
        self.create_subscription(Odometry, "/odometry/global", self.global_samples.append, 50)
        self.create_subscription(Imu, "/gps/course_heading", self.heading_samples.append, 20)
        self.create_subscription(
            Imu, "/localization/orientation", self.orientation_samples.append, 20
        )
        self.create_subscription(Odometry, "/odometry/gps", self.gps_samples.append, 50)
        self.create_subscription(
            PointCloud2, "/obstacles_cloud", self.obstacle_clouds.append,
            qos_profile_sensor_data,
        )
        self.create_subscription(LaserScan, "/scan", self.scans.append, qos_profile_sensor_data)
        self.create_subscription(
            LaserScan,
            "/scan_clean",
            lambda message: self.clean_scans.append((message, time.monotonic())),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray, "/navigation_startup/diagnostics", self._on_startup, 10
        )
        self.create_subscription(
            Twist,
            "/cmd_vel_safe",
            lambda message: self.safe_commands.append((message, time.monotonic())),
            10,
        )
        self.create_subscription(
            CmdVelFinal,
            "/cmd_vel_final",
            lambda message: self.final_commands.append((message, time.monotonic())),
            10,
        )
        self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", self.local_costmaps.append, 10
        )
        self.create_subscription(
            Costmap, "/global_costmap/costmap_raw", self.global_costmaps.append, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.from_ll = self.create_client(FromLL, "/fromLL")
        self.create_timer(0.05, self._publish_inputs)

    def _on_startup(self, message: DiagnosticArray) -> None:
        for status in message.status:
            if status.name == "navigation_startup":
                self.startup_values = {
                    item.key: item.value for item in status.values
                }

    def set_cloud_mode(self, mode: str) -> None:
        assert mode in ("clear", "obstacle")
        self.cloud_mode = mode
        self.cloud_enabled = True

    def stop_cloud(self) -> None:
        self.cloud_enabled = False

    def set_command_enabled(self, enabled: bool) -> None:
        self.command_enabled = enabled

    def wait_for(self, predicate, timeout_s: float, description: str) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.05)
        raise AssertionError(f"timeout waiting for {description}")

    @staticmethod
    def _lidar_point_for_output(
        x: float, y: float, z: float
    ) -> tuple[float, float, float]:
        """Invert the frozen base_link -> lidar_link transform."""
        pitch = 0.1745
        dx, dz = x - 0.92, z - 0.65
        return (
            math.cos(pitch) * dx - math.sin(pitch) * dz,
            y,
            math.sin(pitch) * dx + math.cos(pitch) * dz,
        )

    def _cloud(self, stamp) -> PointCloud2:
        if self.cloud_mode == "clear":
            desired = [
                (12.0, -0.05, 0.8),
                (12.0, -0.02, 0.8),
                (12.0, 0.02, 0.8),
                (12.0, 0.05, 0.8),
            ]
        else:
            desired = [
                (0.5, y, 0.8)
                for y in (-0.02, -0.015, -0.01, -0.005, 0.0, 0.005, 0.01, 0.015, 0.02)
            ]
        header = Header()
        header.stamp = stamp
        header.frame_id = "lidar_link"
        points = [self._lidar_point_for_output(*point) for point in desired]
        return point_cloud2.create_cloud_xyz32(header, points)

    def _publish_inputs(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        stamp = self.get_clock().now().to_msg()
        elapsed = max(0.0, (now_ns - self._start_ns) / 1.0e9)
        longitude = DATUM_LON + elapsed / (
            111320.0 * math.cos(math.radians(DATUM_LAT))
        )

        drive = DriveTelemetry()
        drive.stamp = stamp
        drive.ready = True
        drive.fresh = True
        drive.drive_enabled = True
        drive.estop = False
        drive.reverse_requested = False
        drive.speed_valid = True
        drive.steer_valid = True
        drive.control_source = "synthetic_integration_fixture"
        drive.speed_mps_measured = 1.0
        drive.steer_deg_measured = 0.0
        drive.brake_applied_pct = 0
        self.drive_pub.publish(drive)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        imu.orientation_covariance[0] = -1.0
        imu.angular_velocity.z = 0.0
        imu.angular_velocity_covariance[0] = 0.01
        imu.linear_acceleration_covariance[0] = 0.1
        self.imu_pub.publish(imu)

        fix = NavSatFix()
        fix.header.stamp = stamp
        fix.header.frame_id = "gps_link"
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = DATUM_LAT
        fix.longitude = longitude
        fix.altitude = 0.0
        fix.position_covariance[0] = 0.05
        fix.position_covariance[4] = 0.05
        fix.position_covariance[8] = 1.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.gps_pub.publish(fix)

        status = GnssRtkStatus()
        status.header.stamp = stamp
        status.fix_quality = GnssRtkStatus.RTK_FIXED
        status.acquisition_state = GnssRtkStatus.ACQUISITION_RECEIVING
        status.delivery_backend = GnssRtkStatus.BACKEND_DISABLED
        status.delivery_state = GnssRtkStatus.DELIVERY_DISABLED
        status.corrections_fresh = True
        status.correction_age_s = 0.0
        self.rtk_pub.publish(status)

        if self.cloud_enabled:
            self.cloud_pub.publish(self._cloud(stamp))
        if self.command_enabled:
            command = Twist()
            command.linear.x = 1.0
            self.cmd_pub.publish(command)


def _start_launch(command: tuple[str, ...], log_path: Path) -> tuple[subprocess.Popen, object]:
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = _DOMAIN_ID
    environment["RCUTILS_COLORIZED_OUTPUT"] = "0"
    log_handle = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        list(command),
        env=environment,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    return process, log_handle


def _wait_for_transform(
    harness: RealStackIntegrationHarness,
    target: str,
    source: str,
    timeout_s: float = 20.0,
) -> None:
    def available() -> bool:
        try:
            harness.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            return True
        except TransformException:
            return False

    harness.wait_for(available, timeout_s, f"TF {target} <- {source}")


def _wait_for_service(harness: Node, client, timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if client.service_is_ready():
            return
        time.sleep(0.05)
    raise AssertionError(f"timeout waiting for {description} service")


def _get_state(harness: Node, node_name: str) -> State:
    client = harness.create_client(GetState, f"/{node_name}/get_state")
    _wait_for_service(harness, client, 20.0, node_name)
    future = client.call_async(GetState.Request())
    deadline = time.monotonic() + 10.0
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    assert future.done(), f"timeout waiting for {node_name} state response"
    response = future.result()
    assert response is not None
    return response.current_state


def _scan_has_obstacle(scan: LaserScan) -> bool:
    finite = [value for value in scan.ranges if math.isfinite(value)]
    return bool(finite) and min(finite) <= 0.8


def _publishers(harness: Node, topic: str) -> set[str]:
    return {info.node_name for info in harness.get_publishers_info_by_topic(topic)}


def _run_integration(log_root: Path, runtime_dir: Path) -> None:
    rclpy.init(domain_id=int(_DOMAIN_ID))
    harness = RealStackIntegrationHarness()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    processes: list[tuple[subprocess.Popen, object]] = []
    try:
        for name, command in LAUNCHES:
            processes.append(_start_launch(command, log_root / f"{name}.log"))
        navigation_command = NAVIGATION_LAUNCH[1] + (
            "use_keepout:=true",
            f"zones_runtime_dir:={runtime_dir}",
        )
        processes.append(_start_launch(navigation_command, log_root / "navigation.log"))

        harness.wait_for(
            lambda: EXPECTED_NODES.issubset(set(harness.get_node_names())),
            60.0,
            "all product nodes from the five real launches",
        )
        harness.wait_for(
            lambda: len(harness.wheel_samples) >= 5
            and _progressive(harness.wheel_samples),
            30.0,
            "finite progressive /wheel/odometry",
        )
        harness.wait_for(
            lambda: len(harness.local_samples) >= 5
            and all(_finite_odometry(message) for message in harness.local_samples[-5:])
            and _progressive(harness.local_samples),
            30.0,
            "finite progressive /odometry/local",
        )
        assert all(_finite_odometry(message) for message in harness.wheel_samples[-5:])

        harness.wait_for(
            lambda: len(harness.heading_samples) >= 1,
            35.0,
            "global course heading after the required displacement",
        )
        harness.wait_for(
            lambda: len(harness.orientation_samples) >= 1,
            20.0,
            "global orientation",
        )
        harness.wait_for(
            lambda: len(harness.gps_samples) >= 3,
            30.0,
            "GPS odometry",
        )
        harness.wait_for(
            lambda: len(harness.global_samples) >= 5
            and all(_finite_odometry(message) for message in harness.global_samples[-5:])
            and _progressive(harness.global_samples),
            35.0,
            "finite progressive /odometry/global",
        )

        _wait_for_transform(harness, "odom", "base_footprint")
        _wait_for_transform(harness, "map", "odom")
        _wait_for_transform(harness, "base_footprint", "base_link")
        _wait_for_transform(harness, "base_link", "lidar_link")
        _wait_for_transform(harness, "map", "lidar_link")

        harness.wait_for(
            lambda: len(harness.clean_scans) >= 3
            and all(
                message.header.frame_id == "base_footprint"
                for message, _ in harness.clean_scans[-3:]
            ),
            30.0,
            "perception /scan_clean output",
        )
        assert harness.obstacle_clouds
        assert harness.scans
        tf_publishers = _publishers(harness, "/tf")
        assert {"salus_local_ekf", "salus_global_ekf"}.issubset(tf_publishers)
        assert tf_publishers <= {
            "robot_state_publisher", "salus_local_ekf", "salus_global_ekf",
        }
        static_tf_publishers = _publishers(harness, "/tf_static")
        assert "robot_state_publisher" in static_tf_publishers
        assert static_tf_publishers <= {"robot_state_publisher", "navsat_transform"}

        zones = harness.create_client(GetZonesState, "/zones_manager/get_state")
        _wait_for_service(harness, zones, 20.0, "zones state")
        _wait_for_service(harness, harness.from_ll, 20.0, "/fromLL")
        zones_response = zones.call_async(GetZonesState.Request())
        harness.wait_for(lambda: zones_response.done(), 10.0, "zones state response")
        zones_state = zones_response.result()
        assert zones_state is not None and zones_state.ok and zones_state.mask_ready

        harness.wait_for(
            lambda: harness.startup_values.get("state") == "ACTIVE",
            45.0,
            "navigation startup ACTIVE without /clock",
        )
        for node_name in (
            "planner_server", "controller_server", "bt_navigator", "behavior_server"
        ):
            assert _get_state(harness, node_name).id == State.PRIMARY_STATE_ACTIVE
        harness.wait_for(
            lambda: bool(harness.local_costmaps) and bool(harness.global_costmaps),
            25.0,
            "Nav2 local/global costmaps",
        )

        assert harness.count_publishers("/clock") == 0
        assert _publishers(harness, "/cmd_vel_safe") == {"collision_monitor"}
        assert _publishers(harness, "/cmd_vel_final") == {"nav_command_server"}
        graph_names = set(harness.get_node_names())
        assert not any(
            any(
                token in name.lower()
                for token in ("mavros", "uart", "serial", "rslidar", "ntrip")
            )
            for name in graph_names
        )
        assert "controller_server_node" not in graph_names

        planner = ActionClient(harness, ComputePathToPose, "/compute_path_to_pose")
        harness.wait_for(planner.server_is_ready, 20.0, "ComputePathToPose action")
        start = harness.global_samples[-1]
        goal = ComputePathToPose.Goal()
        goal.use_start = True
        goal.planner_id = "GridBased"
        goal.start.header.stamp = harness.get_clock().now().to_msg()
        goal.start.header.frame_id = "map"
        goal.start.pose = start.pose.pose
        goal.goal.header.stamp = harness.get_clock().now().to_msg()
        goal.goal.header.frame_id = "map"
        goal.goal.pose.position.x = start.pose.pose.position.x + 8.0
        goal.goal.pose.position.y = start.pose.pose.position.y
        goal.goal.pose.orientation.w = 1.0
        send_future = planner.send_goal_async(goal)
        harness.wait_for(lambda: send_future.done(), 20.0, "ComputePathToPose goal response")
        handle = send_future.result()
        assert handle is not None and handle.accepted
        result_future = handle.get_result_async()
        harness.wait_for(lambda: result_future.done(), 25.0, "ComputePathToPose result")
        result = result_future.result().result
        assert result is not None
        if hasattr(result, "error_code"):
            assert result.error_code == ComputePathToPose.Result.NONE
        assert result.path.header.frame_id == "map"
        assert result.path.poses
        assert all(
            all(math.isfinite(value) for value in (
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w,
            ))
            for pose in result.path.poses
        )

        harness.set_command_enabled(True)
        harness.wait_for(
            lambda: any(
                message.source == CmdVelFinal.SOURCE_AUTO
                and message.twist.linear.x > 0.5
                for message, _ in harness.final_commands
            ),
            15.0,
            "clear-cloud forward command through final authority",
        )

        harness.set_cloud_mode("obstacle")
        harness.wait_for(
            lambda: any(_scan_has_obstacle(message) for message, _ in harness.clean_scans),
            15.0,
            "obstacle propagated to /scan_clean",
        )
        harness.safe_commands.clear()
        harness.wait_for(
            lambda: any(
                message.linear.x == 0.0 and message.angular.z == 0.0
                for message, _ in harness.safe_commands
            ),
            10.0,
            "Collision Monitor safe stop",
        )
        harness.final_commands.clear()
        obstacle_boundary = time.monotonic()
        harness.wait_for(
            lambda: any(
                message.twist.linear.x == 0.0 and message.twist.angular.z == 0.0
                for message, _ in harness.final_commands
            ),
            10.0,
            "arbitrated final safe stop",
        )
        assert not any(
            message.source == CmdVelFinal.SOURCE_AUTO
            and message.twist.linear.x > 0.0
            and received_at >= obstacle_boundary
            for message, received_at in harness.final_commands
        )

        harness.final_commands.clear()
        harness.stop_cloud()
        stale_started = time.monotonic()
        timeout_boundary = stale_started + 1.1
        harness.wait_for(
            lambda: harness.clean_scans
            and time.monotonic() - harness.clean_scans[-1][1] > 1.1,
            4.0,
            "stale perception source after stopping /scan_3d",
        )
        harness.wait_for(
            lambda: time.monotonic() - stale_started > 1.8,
            3.0,
            "source timeout safety boundary",
        )
        assert not any(
            message.source == CmdVelFinal.SOURCE_AUTO
            and message.twist.linear.x > 0.0
            and received_at >= timeout_boundary
            for message, received_at in harness.final_commands
        )
    finally:
        for process, log_handle in reversed(processes):
            _stop_process(process)
            log_handle.close()
        executor.remove_node(harness)
        harness.destroy_node()
        executor.shutdown()
        spinner.join(timeout=5.0)
        if rclpy.ok():
            rclpy.shutdown()


def test_real_stack_integration_pc_runtime(tmp_path: Path) -> None:
    """Exercise real owners end-to-end using only synthetic physical inputs."""
    with tempfile.TemporaryDirectory(
        prefix="salus-real-stack-integration-", dir="/tmp"
    ) as temp:
        root = Path(temp)
        _run_integration(root, root / "empty-zones")
