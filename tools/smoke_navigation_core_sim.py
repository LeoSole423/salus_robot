#!/usr/bin/env python3
"""Exercise LL goals, Nav2 movement, cancellation and manual takeover in simulation."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
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

    @staticmethod
    def goal_request(east_m: float) -> SetNavGoalLL.Request:
        request = SetNavGoalLL.Request()
        request.lat = DATUM_LAT
        request.lon = DATUM_LON + east_m / (111_320.0 * math.cos(math.radians(DATUM_LAT)))
        request.yaw_deg = 0.0
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


def send_goal(node: NavigationSmoke, east_m: float) -> None:
    response = call(node, node.goal, node.goal_request(east_m), "goal service unavailable")
    if not response.ok:
        raise RuntimeError(f"goal rejected: {response.error}")


def get_state(node: NavigationSmoke):
    return call(node, node.state, GetNavState.Request(), "state service unavailable")


def main() -> int:
    rclpy.init()
    node = NavigationSmoke()
    try:
        wait_for(node, lambda: node.odom and node.telemetry, 20.0, "global odometry or telemetry unavailable")
        start_x = node.odom[-1].pose.pose.position.x
        send_goal(node, 7.0)
        wait_for(node, lambda: get_state(node).goal_active, 8.0, "goal was not accepted by Nav2")
        wait_for(
            node,
            lambda: any(message.source == CmdVelFinal.SOURCE_AUTO and message.twist.linear.x > 0.1 for message in node.final),
            10.0,
            "Nav2 did not produce an automatic command",
        )
        wait_for(node, lambda: node.odom[-1].pose.pose.position.x > start_x + 1.0, 18.0, "robot did not advance toward the LL goal")
        wait_for(node, lambda: not get_state(node).goal_active, 30.0, "short goal did not finish")
        if not any(message.nav_result_text == "succeeded" for message in node.telemetry):
            raise RuntimeError("goal did not report success")

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
        print("Navigation core simulation smoke test passed")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
