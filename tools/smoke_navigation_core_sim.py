#!/usr/bin/env python3
"""Exercise LL goals, Nav2 movement, cancellation and manual takeover in simulation."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_localization.srv import FromLL
from salus_interfaces.msg import CmdVelFinal, NavTelemetry
from salus_interfaces.srv import CancelNavGoal, GetNavState, SetManualMode, SetNavGoalLL


DATUM_LAT = -31.4858037
DATUM_LON = -64.2410570


class NavigationSmoke(Node):
    def __init__(self) -> None:
        super().__init__("navigation_core_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom: list[Odometry] = []
        self.plans: list[Path] = []
        self.final: list[CmdVelFinal] = []
        self.telemetry: list[NavTelemetry] = []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        self.create_subscription(Path, "/plan", self.plans.append, 10)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self.final.append, 10)
        self.create_subscription(NavTelemetry, "/nav_command_server/telemetry", self.telemetry.append, 10)
        self.goal = self.create_client(SetNavGoalLL, "/nav_command_server/set_goal_ll")
        self.cancel = self.create_client(CancelNavGoal, "/nav_command_server/cancel_goal")
        self.state = self.create_client(GetNavState, "/nav_command_server/get_state")
        self.manual = self.create_client(SetManualMode, "/nav_command_server/set_manual_mode")
        self.fromll = self.create_client(FromLL, "/fromLL")

    @staticmethod
    def goal_request(x_m: float, y_m: float, yaw_rad: float) -> SetNavGoalLL.Request:
        request = SetNavGoalLL.Request()
        request.lat = DATUM_LAT + y_m / 111_320.0
        request.lon = DATUM_LON + x_m / (111_320.0 * math.cos(math.radians(DATUM_LAT)))
        request.yaw_deg = math.degrees(yaw_rad)
        return request


def wait_for(node: NavigationSmoke, predicate, timeout_s: float, error: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise RuntimeError(error)


def call(node: NavigationSmoke, client, request, error: str):
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeError(error)
    future = client.call_async(request)
    wait_for(node, future.done, 5.0, error)
    return future.result()


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
    try:
        wait_for(node, lambda: node.odom and node.telemetry, 20.0, "global odometry or telemetry unavailable")
        # The integrated smoke has already exercised manual control and braking.
        # Restore the navigation precondition explicitly before sending its goal.
        response = call(node, node.cancel, CancelNavGoal.Request(), "cancel service unavailable")
        if not response.ok:
            raise RuntimeError(response.error)
        response = call(node, node.manual, SetManualMode.Request(enabled=False), "manual-mode service unavailable")
        if not response.ok or response.enabled_after:
            raise RuntimeError("automatic mode was not restored before navigation")
        start = node.odom[-1]
        target_x, target_y = send_goal(node, 7.0)
        wait_for(node, lambda: get_state(node).goal_active, 8.0, "goal was not accepted by Nav2")
        wait_for(
            node,
            lambda: any(message.source == CmdVelFinal.SOURCE_AUTO and message.twist.linear.x > 0.1 for message in node.final),
            10.0,
            "Nav2 did not produce an automatic command",
        )
        wait_for(node, lambda: distance_from(start, node.odom[-1]) > 1.0, 18.0, "robot did not advance toward the LL goal")
        wait_for(node, lambda: not get_state(node).goal_active, 30.0, "short goal did not finish")
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
        response = call(node, node.cancel, CancelNavGoal.Request(), "cancel service unavailable")
        if not response.ok:
            raise RuntimeError(response.error)
        wait_for(node, lambda: not get_state(node).goal_active, 5.0, "cancelled goal remained active")
        wait_for(node, lambda: any(message.twist.linear.x == 0.0 and message.brake_pct == 100 for message in node.final), 4.0, "cancel did not issue a safe brake")

        send_goal(node, 25.0)
        wait_for(node, lambda: get_state(node).goal_active, 8.0, "goal before manual takeover was not accepted")
        response = call(node, node.manual, SetManualMode.Request(enabled=True), "manual-mode service unavailable")
        if not response.ok or not response.enabled_after:
            raise RuntimeError("manual takeover was rejected")
        wait_for(node, lambda: not get_state(node).goal_active, 5.0, "manual takeover did not cancel goal")
        response = call(node, node.manual, SetManualMode.Request(enabled=False), "manual-mode service unavailable")
        if not response.ok or response.enabled_after:
            raise RuntimeError("automatic mode was not restored")
        response = call(node, node.cancel, CancelNavGoal.Request(), "cancel service unavailable")
        if not response.ok:
            raise RuntimeError(response.error)
        print("Navigation core simulation smoke test passed")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
