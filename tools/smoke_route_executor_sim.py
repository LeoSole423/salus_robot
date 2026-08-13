#!/usr/bin/env python3
"""Smoke the route executor through its public ROS contracts."""
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from salus_interfaces.msg import CmdVelFinal, NavTelemetry, PathHealth
from salus_interfaces.srv import CancelRouteMission, GetRouteMissionState, SetRouteMissionLL

LAT, LON = -31.4858037, -64.2410570


class Smoke(Node):
    def __init__(self):
        super().__init__("route_executor_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom, self.mission_paths, self.chunks, self.final = [], [], [], []
        self.path_health, self.telemetry = [], []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        self.create_subscription(Path, "/route_executor/mission_path", self.mission_paths.append, 10)
        self.create_subscription(Path, "/route_executor/active_chunk_path", self.chunks.append, 10)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self.final.append, 10)
        self.create_subscription(PathHealth, "/path_health", self.path_health.append, 10)
        self.create_subscription(
            NavTelemetry, "/nav_command_server/telemetry", self.telemetry.append, 10
        )
        self.set = self.create_client(SetRouteMissionLL, "/route_executor/set_route_mission_ll")
        self.state = self.create_client(GetRouteMissionState, "/route_executor/get_route_mission_state")
        self.cancel = self.create_client(CancelRouteMission, "/route_executor/cancel_route_mission")


def wait(node, predicate, timeout, error):
    until = time.monotonic() + timeout
    while time.monotonic() < until:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate(): return
    raise RuntimeError(error)


def call(node, client, request):
    if not client.wait_for_service(timeout_sec=8): raise RuntimeError("route service unavailable")
    future = client.call_async(request)
    wait(node, future.done, 5, "route service timed out")
    return future.result()


def request_from_pose(pose, *, loop=False):
    yaw = math.atan2(2 * pose.orientation.w * pose.orientation.z, 1 - 2 * pose.orientation.z ** 2)
    x, y = pose.position.x, pose.position.y
    values = [(x + distance * math.cos(yaw), y + distance * math.sin(yaw)) for distance in (3, 6, 9)]
    request = SetRouteMissionLL.Request()
    request.lats = [LAT + point_y / 111_320.0 for _, point_y in values]
    request.lons = [LON + point_x / (111_320.0 * math.cos(math.radians(LAT))) for point_x, _ in values]
    request.yaws_deg = [math.degrees(yaw)] * len(values)
    request.loop, request.leg_spacing_m = loop, 2.0
    request.chunk_span_m, request.chunk_max_waypoints = 4.5, 3
    return request


def main():
    rclpy.init(); node = Smoke()
    try:
        wait(node, lambda: node.odom, 20, "global odometry unavailable")
        result = call(node, node.set, request_from_pose(node.odom[-1].pose.pose))
        if not result.ok: raise RuntimeError(result.error)
        wait(
            node,
            lambda: call(node, node.state, GetRouteMissionState.Request()).status == "ACTIVE",
            12,
            "route preparation did not become ACTIVE",
        )
        wait(node, lambda: node.mission_paths and node.chunks, 10, "route debug paths unavailable")
        try:
            wait(node, lambda: call(node, node.state, GetRouteMissionState.Request()).reached_checkpoint_count >= 1, 35, "route did not reach first checkpoint")
        except RuntimeError as exc:
            state = call(node, node.state, GetRouteMissionState.Request())
            health = node.path_health[-1].reason if node.path_health else "unavailable"
            telemetry = node.telemetry[-1] if node.telemetry else None
            nav_result = "unavailable" if telemetry is None else telemetry.nav_result_text
            raise RuntimeError(
                f"{exc}; status={state.status}; reason={state.blocked_reason_text!r}; "
                f"path_health={health}; nav_result={nav_result!r}"
            ) from exc
        state = call(node, node.state, GetRouteMissionState.Request())
        if state.active_chunk_size > state.chunk_max_waypoints: raise RuntimeError("chunk exceeds configured waypoint limit")
        if not any(message.source == CmdVelFinal.SOURCE_AUTO for message in node.final): raise RuntimeError("route did not reach command chain")
        result = call(node, node.cancel, CancelRouteMission.Request())
        if not result.ok: raise RuntimeError(result.error)
        wait(node, lambda: call(node, node.state, GetRouteMissionState.Request()).status == "CANCELLED", 5, "route was not cancelled")
        print("Route executor simulation smoke test passed")
        return 0
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
