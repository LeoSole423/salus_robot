#!/usr/bin/env python3
"""Smoke the route executor through its public ROS contracts."""
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from robot_localization.srv import FromLL
from salus_interfaces.msg import CmdVelFinal, NavEvent, NavTelemetry, PathHealth
from salus_interfaces.srv import (
    CancelRouteMission, GetRouteMissionState, SetNavGoalLL, SetRouteMissionLL,
)
from smoke_runtime import (
    AsyncServicePoller, SmokeRuntime, finite_odometry, has_increasing_stamps, stamp_ns,
)
from tf2_ros import Buffer, TransformListener

LAT, LON = -31.4858037, -64.2410570


def startup_evidence_is_ready(evidence):
    """Pure readiness decision; kept separate for deterministic harness tests."""
    return (
        all(evidence["actions"].values())
        and all(evidence["services"].values())
        and evidence["odometry_progressive"]
        and evidence["odometry_finite"]
        and evidence["telemetry_messages"] >= 2
        and evidence["bt_state"] == "active"
    )


class Smoke(Node):
    def __init__(self):
        super().__init__("route_executor_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom, self.mission_paths, self.chunks, self.final = [], [], [], []
        self.path_health, self.telemetry, self.events = [], [], []
        self.global_costmaps, self.local_costmaps = [], []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        self.create_subscription(NavPath, "/route_executor/mission_path", self.mission_paths.append, 10)
        self.create_subscription(NavPath, "/route_executor/active_chunk_path", self.chunks.append, 10)
        self.create_subscription(CmdVelFinal, "/cmd_vel_final", self.final.append, 10)
        self.create_subscription(PathHealth, "/path_health", self.path_health.append, 10)
        self.create_subscription(
            NavTelemetry, "/nav_command_server/telemetry", self.telemetry.append, 10
        )
        self.create_subscription(NavEvent, "/nav_command_server/events", self.events.append, 10)
        self.create_subscription(
            Costmap, "/global_costmap/costmap_raw", self.global_costmaps.append, 10
        )
        self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", self.local_costmaps.append, 10
        )
        self.set = self.create_client(SetRouteMissionLL, "/route_executor/set_route_mission_ll")
        self.state = self.create_client(GetRouteMissionState, "/route_executor/get_route_mission_state")
        self.cancel = self.create_client(CancelRouteMission, "/route_executor/cancel_route_mission")
        self.nav_goal = self.create_client(SetNavGoalLL, "/nav_command_server/set_goal_ll")
        self.fromll = self.create_client(FromLL, "/fromLL")
        self.navigate_action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_action = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.follow_action = ActionClient(self, FollowPath, "/follow_path")
        self.bt_state = self.create_client(GetState, "/bt_navigator/get_state")
        self.bt_state_future = None
        self.bt_state_requested_at = 0.0
        self.bt_state_label = "unavailable"
        self.bt_state_requests = 0
        self.bt_state_timeouts = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def poll_bt_state(self):
        now = time.monotonic()
        if self.bt_state_future is not None:
            if self.bt_state_future.done():
                response = self.bt_state_future.result()
                self.bt_state_label = (
                    response.current_state.label if response is not None else "invalid response"
                )
                self.bt_state_future = None
            elif now - self.bt_state_requested_at >= 2.0:
                self.bt_state_future.cancel()
                self.bt_state_future = None
                self.bt_state_timeouts += 1
        if self.bt_state_future is None and self.bt_state.service_is_ready():
            self.bt_state_future = self.bt_state.call_async(GetState.Request())
            self.bt_state_requested_at = now
            self.bt_state_requests += 1

    def startup_evidence(self):
        odom_stamps = [stamp_ns(message) for message in self.odom[-2:]]
        return {
            "actions": {
                "navigate_to_pose": self.navigate_action.server_is_ready(),
                "compute_path_to_pose": self.plan_action.server_is_ready(),
                "follow_path": self.follow_action.server_is_ready(),
            },
            "services": {
                "route_set": self.set.service_is_ready(),
                "route_state": self.state.service_is_ready(),
                "route_cancel": self.cancel.service_is_ready(),
                "fromLL": self.fromll.service_is_ready(),
                "nav_goal": self.nav_goal.service_is_ready(),
                "bt_lifecycle": self.bt_state.service_is_ready(),
            },
            "odometry_messages": len(self.odom),
            "odometry_timestamps_ns": odom_stamps,
            "odometry_progressive": has_increasing_stamps(self.odom),
            "odometry_finite": bool(self.odom) and finite_odometry(self.odom[-1]),
            "telemetry_messages": len(self.telemetry),
            "bt_state": self.bt_state_label,
            "bt_state_requests": self.bt_state_requests,
            "bt_state_timeouts": self.bt_state_timeouts,
        }

    def startup_ready(self):
        evidence = self.startup_evidence()
        return startup_evidence_is_ready(evidence)


def wait(node, predicate, timeout, error, **kwargs):
    node.runtime.wait(error, predicate, timeout, **kwargs)


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


def mission_is_active_or_raise(poller):
    if poller.latest is None:
        return False
    if poller.latest.status in ("PAUSED", "ABORTED", "CANCELLED"):
        raise RuntimeError(
            f"route entered {poller.latest.status}: {poller.latest.blocked_reason_text}"
        )
    return poller.latest.status == "ACTIVE"


def mission_reached_checkpoint_or_raise(poller):
    if poller.latest is None:
        return False
    if poller.latest.status in ("PAUSED", "ABORTED", "CANCELLED"):
        raise RuntimeError(
            f"route entered {poller.latest.status}: {poller.latest.blocked_reason_text}"
        )
    return poller.latest.reached_checkpoint_count >= 1


def main():
    rclpy.init(); node = Smoke()
    runtime = SmokeRuntime(
        node, "routes-free-world",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "route_probe.json",
        global_timeout_s=210.0,
    )
    node.runtime = runtime
    success = False
    failure = None
    state_poller = AsyncServicePoller(
        node.state, GetRouteMissionState.Request, interval_s=0.5, response_timeout_s=8.0
    )
    acceptance_latency_s = None
    try:
        # Full-stack discovery happens concurrently. A single startup budget
        # prevents a slow early dependency from consuming several sequential
        # readiness windows while preserving short functional timeouts below.
        wait(
            node, node.startup_ready, 40, "route startup readiness unavailable",
            stimulate=node.poll_bt_state,
            observe=node.startup_evidence,
        )
        wait(
            node,
            lambda: (
                len(node.global_costmaps) >= 2
                and node.global_costmaps[-1].header.frame_id == "map"
                and node.global_costmaps[-1].metadata.size_x > 0
                and node.global_costmaps[-1].metadata.size_y > 0
                and bool(node.global_costmaps[-1].data)
            ),
            40,
            "global costmap data unavailable",
            observe=lambda: {
                "messages": len(node.global_costmaps),
                "frame": getattr(node.global_costmaps[-1].header, "frame_id", "") if node.global_costmaps else "",
                "cells": len(node.global_costmaps[-1].data) if node.global_costmaps else 0,
            },
        )
        # Require fresh costmaps after activation. This gives Nav2's own TF
        # buffers time to join the graph instead of relying on this probe's TF
        # buffer becoming ready first.
        node.global_costmaps.clear()
        node.local_costmaps.clear()
        wait(
            node,
            lambda: len(node.global_costmaps) >= 2 and len(node.local_costmaps) >= 2,
            15,
            "fresh local/global costmaps unavailable after Nav2 activation",
            observe=lambda: {
                "global": len(node.global_costmaps), "local": len(node.local_costmaps)
            },
        )
        runtime.wait_transform(
            "map to base_footprint", node.tf_buffer, "map", "base_footprint", timeout_s=15.0
        )
        initial_state = call(node, node.state, GetRouteMissionState.Request())
        if not initial_state.ok:
            raise RuntimeError(f"route state handshake failed: {initial_state.error}")
        accepted_at = time.monotonic()
        result = call(node, node.set, request_from_pose(node.odom[-1].pose.pose))
        acceptance_latency_s = time.monotonic() - accepted_at
        if not result.ok: raise RuntimeError(result.error)
        wait(
            node,
            lambda: mission_is_active_or_raise(state_poller),
            12,
            "route preparation did not become ACTIVE",
            stimulate=state_poller.poll,
            observe=lambda: {
                **state_poller.evidence(),
                "status": getattr(state_poller.latest, "status", "unavailable"),
            },
        )
        wait(node, lambda: node.mission_paths and node.chunks, 10, "route debug paths unavailable")
        try:
            wait(
                node,
                lambda: mission_reached_checkpoint_or_raise(state_poller),
                35,
                "route did not reach first checkpoint",
                stimulate=state_poller.poll,
                observe=lambda: {
                    **state_poller.evidence(),
                    "status": getattr(state_poller.latest, "status", "unavailable"),
                    "reached": getattr(state_poller.latest, "reached_checkpoint_count", -1),
                },
            )
        except RuntimeError as exc:
            state = state_poller.latest or initial_state
            health = node.path_health[-1].reason if node.path_health else "unavailable"
            telemetry = node.telemetry[-1] if node.telemetry else None
            nav_result = "unavailable" if telemetry is None else telemetry.nav_result_text
            event = node.events[-1] if node.events else None
            event_text = "unavailable" if event is None else f"{event.code}: {event.message}"
            raise RuntimeError(
                f"{exc}; status={state.status}; reason={state.blocked_reason_text!r}; "
                f"path_health={health}; nav_result={nav_result!r}; last_event={event_text!r}"
            ) from exc
        state = state_poller.latest
        if state.active_chunk_size:
            target = int(state.current_target_index)
            if target >= len(state.mission_key_flags) or not state.mission_key_flags[target]:
                raise RuntimeError("active chunk ends at a synthetic point")
        if not any(message.source == CmdVelFinal.SOURCE_AUTO for message in node.final): raise RuntimeError("route did not reach command chain")
        result = call(node, node.cancel, CancelRouteMission.Request())
        if not result.ok: raise RuntimeError(result.error)
        wait(
            node,
            lambda: state_poller.latest is not None and state_poller.latest.status == "CANCELLED",
            5,
            "route was not cancelled",
            stimulate=state_poller.poll,
            observe=lambda: {
                **state_poller.evidence(),
                "status": getattr(state_poller.latest, "status", "unavailable"),
            },
        )
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
            "global_costmaps": len(node.global_costmaps),
            "local_costmaps": len(node.local_costmaps),
            "set_route_acceptance_latency_s": acceptance_latency_s,
            "state_poller": state_poller.evidence(),
            "last_status": getattr(state_poller.latest, "status", "unavailable"),
            "last_path_health": getattr(node.path_health[-1], "reason", "unavailable") if node.path_health else "unavailable",
            "last_nav_result": getattr(node.telemetry[-1], "nav_result_text", "unavailable") if node.telemetry else "unavailable",
            "last_nav_event": ({"code": node.events[-1].code, "message": node.events[-1].message} if node.events else None),
        })
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
