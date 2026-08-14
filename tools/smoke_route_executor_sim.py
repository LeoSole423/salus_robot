#!/usr/bin/env python3
"""Smoke the route executor through its public ROS contracts."""
import math
import os
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from salus_interfaces.msg import CmdVelFinal, NavEvent, NavTelemetry, PathHealth
from salus_interfaces.srv import CancelRouteMission, GetRouteMissionState, SetRouteMissionLL
from smoke_runtime import SmokeRuntime

LAT, LON = -31.4858037, -64.2410570


class Smoke(Node):
    def __init__(self):
        super().__init__("route_executor_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom, self.mission_paths, self.chunks, self.final = [], [], [], []
        self.path_health, self.telemetry, self.events = [], [], []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        self.create_subscription(Path, "/route_executor/mission_path", self.mission_paths.append, 10)
        self.create_subscription(Path, "/route_executor/active_chunk_path", self.chunks.append, 10)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self.final.append, 10)
        self.create_subscription(PathHealth, "/path_health", self.path_health.append, 10)
        self.create_subscription(
            NavTelemetry, "/nav_command_server/telemetry", self.telemetry.append, 10
        )
        self.create_subscription(NavEvent, "/nav_command_server/events", self.events.append, 10)
        self.set = self.create_client(SetRouteMissionLL, "/route_executor/set_route_mission_ll")
        self.state = self.create_client(GetRouteMissionState, "/route_executor/get_route_mission_state")
        self.cancel = self.create_client(CancelRouteMission, "/route_executor/cancel_route_mission")


def wait(node, predicate, timeout, error):
    node.runtime.wait(error, predicate, timeout)


def call(node, client, request):
    return node.runtime.call("route service", client, request, timeout_s=8.0)


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
    runtime = SmokeRuntime(
        node, "routes-free-world",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "route_probe.json",
    )
    node.runtime = runtime
    success = False
    failure = None
    try:
        wait(
            node,
            lambda: node.odom and all(
                math.isfinite(value)
                for value in (
                    node.odom[-1].pose.pose.position.x,
                    node.odom[-1].pose.pose.position.y,
                )
            ),
            20,
            "finite global odometry unavailable",
        )
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
            event = node.events[-1] if node.events else None
            event_text = "unavailable" if event is None else f"{event.code}: {event.message}"
            raise RuntimeError(
                f"{exc}; status={state.status}; reason={state.blocked_reason_text!r}; "
                f"path_health={health}; nav_result={nav_result!r}; last_event={event_text!r}"
            ) from exc
        state = call(node, node.state, GetRouteMissionState.Request())
        if state.active_chunk_size > state.chunk_max_waypoints: raise RuntimeError("chunk exceeds configured waypoint limit")
        if not any(message.source == CmdVelFinal.SOURCE_AUTO for message in node.final): raise RuntimeError("route did not reach command chain")
        result = call(node, node.cancel, CancelRouteMission.Request())
        if not result.ok: raise RuntimeError(result.error)
        wait(node, lambda: call(node, node.state, GetRouteMissionState.Request()).status == "CANCELLED", 5, "route was not cancelled")
        print("Route executor simulation smoke test passed")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "odometry": len(node.odom), "mission_paths": len(node.mission_paths),
            "chunks": len(node.chunks), "final_commands": len(node.final),
        })
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
