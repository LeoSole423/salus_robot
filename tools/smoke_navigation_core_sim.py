#!/usr/bin/env python3
"""Exercise LL goals, Nav2 movement, cancellation and manual takeover in simulation."""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_localization.srv import FromLL
from salus_interfaces.msg import (
    CapabilityState,
    CmdVelFinal,
    NavTelemetry,
    PathHealth,
    SystemCapabilities,
    VehicleCommand,
)
from std_msgs.msg import String
from salus_interfaces.srv import CancelNavGoal, GetNavState, SetManualMode, SetNavGoalLL
from sensor_msgs.msg import Imu
from smoke_runtime import SmokeRuntime, subscribe_navigation_startup


DATUM_LAT = -31.4858037
DATUM_LON = -64.2410570


class NavigationSmoke(Node):
    def __init__(self) -> None:
        super().__init__("navigation_core_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom: list[Odometry] = []
        self.raw_odom: list[Odometry] = []
        self.local_odom: list[Odometry] = []
        self.plans: list[NavPath] = []
        self.final: list[CmdVelFinal] = []
        self.telemetry: list[NavTelemetry] = []
        self.path_health: list[PathHealth] = []
        self.raw_commands: list[Twist] = []
        self.safe_commands: list[Twist] = []
        self.vehicle_commands: list[VehicleCommand] = []
        self.controller_status: list[dict] = []
        self.capability_profiles: list[SystemCapabilities] = []
        self.selected_orientations: list[Imu] = []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        self.create_subscription(Odometry, "/odom_raw", self.raw_odom.append, 10)
        self.create_subscription(Odometry, "/odometry/local", self.local_odom.append, 10)
        self.create_subscription(
            Imu,
            "/localization/orientation",
            self.selected_orientations.append,
            10,
        )
        self.create_subscription(NavPath, "/plan", self.plans.append, 10)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self.final.append, 10)
        self.create_subscription(NavTelemetry, "/nav_command_server/telemetry", self.telemetry.append, 10)
        self.create_subscription(PathHealth, "/path_health", self.path_health.append, 10)
        self.create_subscription(Twist, "/cmd_vel", self.raw_commands.append, 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self.safe_commands.append, 10)
        self.create_subscription(
            VehicleCommand, "/vehicle/command_shadow", self.vehicle_commands.append, 10,
        )
        self.create_subscription(String, "/controller/status", self._on_controller_status, 10)
        self.create_subscription(
            SystemCapabilities,
            "/system/capabilities",
            self.capability_profiles.append,
            10,
        )
        self.rviz_goal = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.startup = subscribe_navigation_startup(self)
        self.goal = self.create_client(SetNavGoalLL, "/nav_command_server/set_goal_ll")
        self.cancel = self.create_client(CancelNavGoal, "/nav_command_server/cancel_goal")
        self.state = self.create_client(GetNavState, "/nav_command_server/get_state")
        self.manual = self.create_client(SetManualMode, "/nav_command_server/set_manual_mode")
        self.fromll = self.create_client(FromLL, "/fromLL")
        self.navigate_action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_action = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.follow_action = ActionClient(self, FollowPath, "/follow_path")

    def _on_controller_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.controller_status.append(payload)

    @staticmethod
    def goal_request(x_m: float, y_m: float, yaw_rad: float) -> SetNavGoalLL.Request:
        request = SetNavGoalLL.Request()
        request.lat = DATUM_LAT + y_m / 111_320.0
        request.lon = DATUM_LON + x_m / (111_320.0 * math.cos(math.radians(DATUM_LAT)))
        request.yaw_deg = math.degrees(yaw_rad)
        return request


def wait_for(node: NavigationSmoke, predicate, timeout_s: float, error: str) -> None:
    node.runtime.wait(error, predicate, timeout_s)


def call(
    node: NavigationSmoke, client, request, error: str, timeout_s: float = 8.0,
):
    return node.runtime.call(error, client, request, timeout_s=timeout_s)


def wait_for_action_server(node: NavigationSmoke, client: ActionClient, name: str) -> None:
    node.runtime.wait_action(name, client)


def yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def destination_from_current_pose(message: Odometry, distance_m: float) -> tuple[float, float, float]:
    yaw = yaw_from_odometry(message)
    position = message.pose.pose.position
    return (
        position.x + distance_m * math.cos(yaw),
        position.y + distance_m * math.sin(yaw),
        yaw,
    )


def send_goal(node: NavigationSmoke, distance_m: float) -> tuple[float, float]:
    x_m, y_m, yaw_rad = destination_from_current_pose(node.odom[-1], distance_m)
    request = node.goal_request(x_m, y_m, yaw_rad)
    # Compare against precisely the same NavSat conversion that the command
    # server uses.  A hand-written metres-to-LL approximation is adequate to
    # request a test goal, but is not the authoritative map-space target.
    conversion = FromLL.Request()
    conversion.ll_point.latitude = request.lat
    conversion.ll_point.longitude = request.lon
    conversion.ll_point.altitude = 0.0
    map_point = call(node, node.fromll, conversion, "fromLL service unavailable").map_point
    response = call(node, node.goal, request, "goal service unavailable")
    if not response.ok:
        raise RuntimeError(f"goal rejected: {response.error}")
    return map_point.x, map_point.y


def rviz_goal_from_current_pose(
    node: NavigationSmoke,
    forward_m: float,
    lateral_m: float = 0.0,
    yaw_offset_rad: float = 0.0,
) -> PoseStamped:
    current = node.odom[-1]
    yaw_rad = yaw_from_odometry(current)
    position = current.pose.pose.position
    x_m = position.x + forward_m * math.cos(yaw_rad) - lateral_m * math.sin(yaw_rad)
    y_m = position.y + forward_m * math.sin(yaw_rad) + lateral_m * math.cos(yaw_rad)
    goal_yaw_rad = yaw_rad + yaw_offset_rad
    message = PoseStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.position.x = x_m
    message.pose.position.y = y_m
    message.pose.orientation.z = math.sin(goal_yaw_rad * 0.5)
    message.pose.orientation.w = math.cos(goal_yaw_rad * 0.5)
    return message


def yaw_delta(start_rad: float, current_rad: float) -> float:
    return math.atan2(
        math.sin(current_rad - start_rad),
        math.cos(current_rad - start_rad),
    )


def distance_from(start: Odometry, current: Odometry) -> float:
    start_position, current_position = start.pose.pose.position, current.pose.pose.position
    return math.hypot(current_position.x - start_position.x, current_position.y - start_position.y)


def position_error(current: Odometry, target_x: float, target_y: float) -> float:
    position = current.pose.pose.position
    return math.hypot(position.x - target_x, position.y - target_y)


def get_state(node: NavigationSmoke):
    return call(node, node.state, GetNavState.Request(), "state service unavailable")


def main() -> int:
    rclpy.init()
    node = NavigationSmoke()
    runtime = SmokeRuntime(
        node, "navigation-free-world",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "navigation_probe.json",
    )
    node.runtime = runtime
    success = False
    failure = None
    expect_canonical = os.environ.get("EXPECT_CANONICAL_COMMAND", "0") == "1"
    expect_no_obstacles = os.environ.get("EXPECT_NO_OBSTACLE_DETECTION", "0") == "1"
    try:
        wait_for(
            node, lambda: node.startup.active, 45.0,
            "navigation startup did not become active",
        )
        wait_for(
            node,
            lambda: node.odom and node.raw_odom and node.local_odom and node.telemetry,
            20.0,
            "raw/local/global odometry or telemetry unavailable",
        )
        wait_for(
            node,
            lambda: bool(node.capability_profiles),
            10.0,
            "typed capability profile unavailable",
        )
        if expect_no_obstacles:
            profile = node.capability_profiles[-1]
            capabilities = {
                item.capability_id: item for item in profile.capabilities
            }
            obstacle = capabilities.get("local_obstacle_detection")
            if (
                profile.profile != "no_obstacle_detection"
                or obstacle is None
                or obstacle.state != CapabilityState.STATE_DISABLED_BY_PROFILE
                or obstacle.enabled
                or obstacle.required
            ):
                raise RuntimeError("no-obstacle capability profile is not explicit")
            wait_for(
                node,
                lambda: (
                    node.count_publishers("/scan_clean") == 0
                    and node.count_publishers("/scan_preview") == 0
                    and node.count_publishers("/cmd_vel_safe") == 1
                ),
                5.0,
                "no-obstacle profile started LiDAR output or lost unique safe command authority",
            )
        # Lifecycle state and action discovery are not sufficient evidence on
        # a contended runner.  Wait until every action server in the first BT
        # tick accepts clients before submitting the first high-level goal.
        wait_for_action_server(node, node.navigate_action, "/navigate_to_pose")
        wait_for_action_server(node, node.plan_action, "/compute_path_to_pose")
        wait_for_action_server(node, node.follow_action, "/follow_path")
        # The integrated smoke has already exercised manual control and braking.
        # Restore the navigation precondition explicitly before sending its goal.
        response = call(
            node, node.cancel, CancelNavGoal.Request(),
            "cancel service unavailable", timeout_s=15.0,
        )
        if not response.ok:
            raise RuntimeError(response.error)
        response = call(node, node.manual, SetManualMode.Request(enabled=False), "manual-mode service unavailable")
        if not response.ok or response.enabled_after:
            raise RuntimeError("automatic mode was not restored before navigation")
        right_turn_start_yaw = yaw_from_odometry(node.odom[-1])
        right_turn_raw_start_yaw = yaw_from_odometry(node.raw_odom[-1])
        right_turn_local_start_yaw = yaw_from_odometry(node.local_odom[-1])
        right_turn_goal = rviz_goal_from_current_pose(
            node, forward_m=8.0, lateral_m=-8.0, yaw_offset_rad=-math.pi / 2.0,
        )
        node.plans.clear()
        node.raw_commands.clear()
        node.runtime.wait(
            "right-turn RViz goal did not produce a plan",
            lambda: get_state(node).goal_active and bool(node.plans),
            8.0,
            stimulate=lambda: node.rviz_goal.publish(right_turn_goal),
        )
        wait_for(
            node,
            lambda: any(message.angular.z < -0.02 for message in node.raw_commands),
            8.0,
            "right-turn plan did not produce a negative yaw command",
        )
        wait_for(
            node,
            lambda: yaw_delta(
                right_turn_raw_start_yaw, yaw_from_odometry(node.raw_odom[-1])
            ) < -0.03,
            8.0,
            "right-turn command did not produce a negative physical yaw response",
        )
        wait_for(
            node,
            lambda: yaw_delta(
                right_turn_local_start_yaw, yaw_from_odometry(node.local_odom[-1])
            ) < -0.03,
            8.0,
            "right-turn physical motion produced the wrong local-odometry yaw sign",
        )
        wait_for(
            node,
            lambda: yaw_delta(
                right_turn_start_yaw, yaw_from_odometry(node.odom[-1])
            ) < -0.03,
            8.0,
            "right-turn physical motion produced the wrong global-odometry yaw sign",
        )
        response = call(
            node, node.cancel, CancelNavGoal.Request(),
            "cancel service unavailable", timeout_s=15.0,
        )
        if not response.ok:
            raise RuntimeError(response.error)
        wait_for(
            node,
            lambda: not get_state(node).goal_active,
            2.0,
            "cancel service returned before the right-turn goal became terminal",
        )

        start = node.odom[-1]
        rviz_goal = rviz_goal_from_current_pose(node, forward_m=7.0)
        target_x = rviz_goal.pose.position.x
        target_y = rviz_goal.pose.position.y
        node.raw_commands.clear()
        node.safe_commands.clear()
        node.final.clear()
        node.vehicle_commands.clear()
        node.controller_status.clear()
        node.runtime.wait(
            "RViz /goal_pose has no subscriber",
            lambda: node.rviz_goal.get_subscription_count() >= 1,
            8.0,
        )
        node.rviz_goal.publish(rviz_goal)
        wait_for(
            node, lambda: get_state(node).goal_active, 8.0,
            "RViz /goal_pose was not accepted by Nav2")
        try:
            wait_for(
                node,
                lambda: (
                    any(message.linear.x > 0.1 for message in node.raw_commands)
                    and any(message.linear.x > 0.1 for message in node.safe_commands)
                    and any(message.source == CmdVelFinal.SOURCE_AUTO and message.twist.linear.x > 0.1 for message in node.final)
                ),
                10.0,
                "Nav2 command did not traverse raw, safe and final stages",
            )
        except RuntimeError as exc:
            reason = node.path_health[-1].reason if node.path_health else "no PathHealth message"
            raise RuntimeError(f"{exc}; last path health: {reason}") from exc
        if expect_canonical:
            wait_for(
                node,
                lambda: (
                    any(
                        message.source == VehicleCommand.SOURCE_AUTO
                        and message.drive_enabled
                        and not message.emergency_stop
                        and message.brake_ratio < 0.01
                        and message.drive.speed > 0.1
                        and (message.valid_for.sec > 0 or message.valid_for.nanosec > 0)
                        for message in node.vehicle_commands
                    )
                    and any(
                        status.get("input_mode") == "canonical_vehicle_command"
                        and status.get("fresh") is True
                        and str(status.get("source", "")).startswith("canonical_")
                        and float(status.get("command", {}).get("speed_mps", 0.0)) > 0.1
                        for status in node.controller_status
                    )
                ),
                8.0,
                "Nav2 command did not reach a fresh canonical controller input",
            )
        wait_for(node, lambda: distance_from(start, node.odom[-1]) > 1.0, 18.0, "robot did not advance toward the RViz goal")
        wait_for(node, lambda: not get_state(node).goal_active, 30.0, "short goal did not finish")
        wait_for(
            node,
            lambda: bool(node.selected_orientations),
            4.0,
            "course-over-ground selection produced no global orientation",
        )
        # Let the final odometry sample arrive after the action result.  The
        # action result is the authoritative goal-tolerance decision because
        # Nav2 evaluates the map->base_footprint transform, whereas this
        # diagnostic topic is an EKF estimate sampled asynchronously.
        odom_count = len(node.odom)
        wait_for(node, lambda: len(node.odom) >= odom_count + 2, 2.0, "final odometry did not settle")
        final_error = position_error(node.odom[-1], target_x, target_y)
        if not math.isfinite(final_error):
            raise RuntimeError(
                "global odometry became invalid after Nav2 reported success"
            )
        wait_for(
            node,
            lambda: any(message.nav_result_text == "succeeded" for message in node.telemetry),
            4.0,
            "goal did not report success",
        )

        node.final.clear()
        send_goal(node, 25.0)
        wait_for(node, lambda: get_state(node).goal_active, 8.0, "long goal was not accepted")
        response = call(
            node, node.cancel, CancelNavGoal.Request(),
            "cancel service unavailable", timeout_s=15.0,
        )
        if not response.ok:
            raise RuntimeError(response.error)
        wait_for(node, lambda: not get_state(node).goal_active, 5.0, "cancelled goal remained active")
        wait_for(node, lambda: any(message.twist.linear.x == 0.0 and message.brake_pct == 100 for message in node.final), 4.0, "cancel did not issue a safe brake")

        send_goal(node, 25.0)
        wait_for(node, lambda: get_state(node).goal_active, 8.0, "goal before manual takeover was not accepted")
        response = call(node, node.manual, SetManualMode.Request(enabled=True), "manual-mode service unavailable")
        if not response.ok or not response.enabled_after:
            raise RuntimeError("manual takeover was rejected")
        wait_for(
            node,
            lambda: not get_state(node).goal_active,
            15.0,
            "manual takeover gained command authority but Nav2 cancellation did not reach a terminal state",
        )
        response = call(node, node.manual, SetManualMode.Request(enabled=False), "manual-mode service unavailable")
        if not response.ok or response.enabled_after:
            raise RuntimeError("automatic mode was not restored")
        response = call(
            node, node.cancel, CancelNavGoal.Request(),
            "cancel service unavailable", timeout_s=15.0,
        )
        if not response.ok:
            raise RuntimeError(response.error)
        print("Navigation core simulation smoke test passed")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "odometry": len(node.odom), "plans": len(node.plans),
            "raw_odometry": len(node.raw_odom), "local_odometry": len(node.local_odom),
            "final_commands": len(node.final), "telemetry": len(node.telemetry),
            "path_health": len(node.path_health),
            "raw_commands": len(node.raw_commands), "safe_commands": len(node.safe_commands),
            "vehicle_commands": len(node.vehicle_commands),
            "controller_status": len(node.controller_status),
            "expected_command_input": (
                "canonical_vehicle_command" if expect_canonical else "legacy_cmd_vel"
            ),
            "capability_profiles": len(node.capability_profiles),
            "selected_orientations": len(node.selected_orientations),
            "expected_capability_profile": (
                "no_obstacle_detection" if expect_no_obstacles else "obstacle_detection"
            ),
            "navigation_startup": node.startup.snapshot(),
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
