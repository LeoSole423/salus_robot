"""
Synthetic runtime gate for the real Collision Monitor executable.

The launch under test starts only Collision Monitor and its lifecycle manager.
This harness owns no safety node: it only supplies scan, command, and optional
fixture TF data from an isolated ROS domain.
"""

from __future__ import annotations

import argparse
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

# Set the domain before importing or initializing rclpy so no DDS participant
# can be created in the caller's default domain.
_DOMAIN_ID = os.environ.get(
    "SALUS_COLLISION_MONITOR_REAL_DOMAIN_ID",
    str(1 + (os.getpid() % 230)),
)
os.environ["ROS_DOMAIN_ID"] = _DOMAIN_ID

import rclpy  # noqa: E402
from geometry_msgs.msg import TransformStamped, Twist  # noqa: E402
from lifecycle_msgs.msg import State  # noqa: E402
from lifecycle_msgs.srv import GetState  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import qos_profile_sensor_data  # noqa: E402
from sensor_msgs.msg import LaserScan  # noqa: E402
from tf2_ros.static_transform_broadcaster import (  # noqa: E402
    StaticTransformBroadcaster,
)


LAUNCH_COMMAND = [
    "ros2",
    "launch",
    "salus_navigation",
    "collision_monitor_real.launch.py",
]


class CollisionMonitorRuntimeHarness(Node):
    """Publish synthetic inputs and observe the real command boundary."""

    def __init__(self) -> None:
        super().__init__("collision_monitor_real_runtime")
        self._obstacle_range: float | None = None
        self.safe_commands: list[Twist] = []
        self.scan_stamps: list[float] = []
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_pub = self.create_publisher(
            LaserScan, "/scan_clean", qos_profile_sensor_data
        )
        self.state_client = self.create_client(GetState, "/collision_monitor/get_state")
        self.create_subscription(Twist, "/cmd_vel_safe", self.safe_commands.append, 10)
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_timer = self.create_timer(0.05, self._publish_inputs)

    def set_obstacle(self, distance: float | None) -> None:
        self._obstacle_range = distance

    def clear_observed_commands(self) -> None:
        self.safe_commands.clear()

    def _publish_inputs(self) -> None:
        stamp = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "base_footprint"
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan.angle_increment = math.pi / 359.0
        scan.time_increment = 0.0
        scan.scan_time = 0.05
        scan.range_min = 0.4
        scan.range_max = 20.0
        scan.ranges = [float("inf")] * 360
        if self._obstacle_range is not None:
            # A compact forward cluster is sufficient for polygon membership,
            # without involving the RS16/perception pipeline.
            for index in range(176, 185):
                scan.ranges[index] = self._obstacle_range

        command = Twist()
        command.linear.x = 1.0
        self.scan_pub.publish(scan)
        self.cmd_pub.publish(command)
        self.scan_stamps.append(
            float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        )

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(transform)

    def command_publishers(self) -> int:
        return self.count_publishers("/cmd_vel_safe")

    def collision_monitor_active(self) -> bool:
        if not self.state_client.service_is_ready():
            return False
        future = self.state_client.call_async(GetState.Request())
        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done() or future.result() is None:
            return False
        return future.result().current_state.id == State.PRIMARY_STATE_ACTIVE


def _wait_for(predicate, timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise AssertionError(f"timeout waiting for {description}")


def _start_launch(log_path: Path) -> subprocess.Popen:
    log_file = log_path.open("w", encoding="utf-8")
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = _DOMAIN_ID
    environment["RCUTILS_COLORIZED_OUTPUT"] = "0"
    return subprocess.Popen(
        LAUNCH_COMMAND,
        env=environment,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _stop_launch(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(os.getpgid(process.pid), signal.SIGINT)
    try:
        process.wait(timeout=20.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
        process.wait(timeout=10.0)


def _run_runtime(log_path: Path) -> None:
    rclpy.init(domain_id=int(_DOMAIN_ID))
    harness = CollisionMonitorRuntimeHarness()
    executor = SingleThreadedExecutor()
    executor.add_node(harness)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    launch_process = _start_launch(log_path)

    try:
        _wait_for(
            lambda: "collision_monitor" in harness.get_node_names()
            and "lifecycle_manager_collision_monitor_real"
            in harness.get_node_names(),
            20.0,
            "the isolated Collision Monitor launch",
        )
        _wait_for(
            harness.collision_monitor_active,
            20.0,
            "Collision Monitor lifecycle ACTIVE",
        )
        _wait_for(
            lambda: harness.command_publishers() == 1,
            10.0,
            "the unique /cmd_vel_safe publisher",
        )

        harness.set_obstacle(None)
        harness.clear_observed_commands()
        _wait_for(
            lambda: any(command.linear.x > 0.5 for command in harness.safe_commands),
            10.0,
            "a non-stopped output for a clear scan",
        )

        harness.set_obstacle(0.5)
        harness.clear_observed_commands()
        _wait_for(
            lambda: any(
                command.linear.x == 0.0 and command.angular.z == 0.0
                for command in harness.safe_commands
            ),
            10.0,
            "a stopped output for the footprint obstacle",
        )

        harness.set_obstacle(2.5)
        harness.clear_observed_commands()
        _wait_for(
            lambda: any(
                0.0 < command.linear.x < 1.0
                and math.copysign(1.0, command.linear.x) == 1.0
                for command in harness.safe_commands
            ),
            10.0,
            "a reduced forward output for the critical slowdown obstacle",
        )

        assert harness.scan_stamps
        assert all(stamp > 0.0 for stamp in harness.scan_stamps)
        assert harness.command_publishers() == 1
        assert launch_process.poll() is None
        log_contents = log_path.read_text(encoding="utf-8")
        assert not any(
            "[ERROR]" in line or "[FATAL]" in line
            for line in log_contents.splitlines()
        )

        node_names = set(harness.get_node_names())
        forbidden_nodes = {
            "nav_command_server",
            "planner_server",
            "controller_server",
            "bt_navigator",
            "smoother_server",
            "route_executor",
            "patrol_mission_coordinator",
        }
        assert not forbidden_nodes.intersection(node_names)
    finally:
        _stop_launch(launch_process)
        executor.remove_node(harness)
        harness.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_real_collision_monitor_runtime() -> None:
    """Run the real executable against synthetic inputs in an isolated domain."""
    with tempfile.TemporaryDirectory(prefix="salus-collision-monitor-real-test-") as temp_dir:
        _run_runtime(Path(temp_dir) / "collision_monitor_real.log")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-path", type=Path)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    with tempfile.TemporaryDirectory(prefix="salus-collision-monitor-real-") as temp_dir:
        log_path = args.log_path or (Path(temp_dir) / "collision_monitor_real.log")
        try:
            _run_runtime(log_path)
        except Exception as exc:  # pragma: no cover - exercised by runtime failures
            print(f"collision monitor runtime failed: {exc}", file=sys.stderr)
            if log_path.exists():
                print(log_path.read_text(encoding="utf-8")[-12000:], file=sys.stderr)
            return 1
    print(f"collision monitor runtime passed (ROS_DOMAIN_ID={_DOMAIN_ID})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
