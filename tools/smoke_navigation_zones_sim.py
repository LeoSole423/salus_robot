#!/usr/bin/env python3
"""Exercise dynamic GeoJSON keepout masks against the active Nav2 stack."""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, FollowPath, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from salus_interfaces.msg import NavTelemetry, ProjectedKeepoutState
from salus_interfaces.srv import GetZonesState, SetNavGoalLL, SetZonesGeoJson
from std_srvs.srv import Trigger
from smoke_runtime import (
    AsyncServicePoller,
    SmokeRuntime,
    finite_odometry,
    has_increasing_stamps,
    stamp_ns,
    subscribe_navigation_startup,
)


DATUM_LAT, DATUM_LON = -31.4858037, -64.2410570


def startup_evidence_is_ready(evidence):
    """Pure causal startup contract used by the zones smoke and its tests."""
    return (
        evidence["navigation_active"]
        and all(evidence["actions"].values())
        and all(evidence["services"].values())
        and evidence["odometry_progressive"]
        and evidence["odometry_finite"]
        and evidence["telemetry_messages"] >= 2
        and evidence["bt_state"] == "active"
        and evidence["projected_messages"] >= 1
        and evidence["projected_frame"] == "map"
    )


class ZonesSmoke(Node):
    def __init__(self):
        super().__init__("zones_sim_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom: list[Odometry] = []
        self.projected: list[ProjectedKeepoutState] = []
        self.plans: list[NavPath] = []
        self.global_costmaps: list[Costmap] = []
        self.telemetry: list[NavTelemetry] = []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        projected_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ProjectedKeepoutState, "/zones_manager/projected_keepouts",
            self.projected.append, projected_qos,
        )
        self.create_subscription(NavPath, "/plan", self.plans.append, 10)
        self.create_subscription(Costmap, "/global_costmap/costmap_raw", self.global_costmaps.append, 10)
        self.create_subscription(
            NavTelemetry, "/nav_command_server/telemetry", self.telemetry.append, 10
        )
        self.set_zones = self.create_client(SetZonesGeoJson, "/zones_manager/set_geojson")
        self.get_zones = self.create_client(GetZonesState, "/zones_manager/get_state")
        self.reload = self.create_client(Trigger, "/zones_manager/reload_from_disk")
        self.goal = self.create_client(SetNavGoalLL, "/nav_command_server/set_goal_ll")
        self.navigate_action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.plan_action = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.follow_action = ActionClient(self, FollowPath, "/follow_path")
        self.bt_state = self.create_client(GetState, "/bt_navigator/get_state")
        self.bt_state_future = None
        self.bt_state_requested_at = 0.0
        self.bt_state_label = "unavailable"
        self.bt_state_requests = 0
        self.bt_state_timeouts = 0
        self.startup = subscribe_navigation_startup(self)

    def poll_bt_state(self):
        now = time.monotonic()
        if self.bt_state_future is not None:
            if self.bt_state_future.done():
                response = self.bt_state_future.result()
                self.bt_state_label = (
                    response.current_state.label
                    if response is not None
                    else "invalid response"
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
        projected = self.projected[-1] if self.projected else None
        return {
            "navigation_active": self.startup.active,
            "navigation_startup": self.startup.snapshot(),
            "actions": {
                "navigate_to_pose": self.navigate_action.server_is_ready(),
                "compute_path_to_pose": self.plan_action.server_is_ready(),
                "follow_path": self.follow_action.server_is_ready(),
            },
            "services": {
                "zones_set": self.set_zones.service_is_ready(),
                "zones_state": self.get_zones.service_is_ready(),
                "zones_reload": self.reload.service_is_ready(),
                "nav_goal": self.goal.service_is_ready(),
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
            "projected_messages": len(self.projected),
            "projected_frame": projected.header.frame_id if projected is not None else "",
            "projected_polygons": len(projected.polygons) if projected is not None else 0,
        }

    def startup_ready(self):
        return startup_evidence_is_ready(self.startup_evidence())


def wait_for(node, predicate, timeout, message):
    node.runtime.wait(message, predicate, timeout)


def call(node, client, request, message, timeout_s=20.0):
    return node.runtime.call(message, client, request, timeout_s=timeout_s)


def yaw(odometry):
    q = odometry.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def lat_lon(x, y):
    return DATUM_LAT + y / 111_320.0, DATUM_LON + x / (111_320.0 * math.cos(math.radians(DATUM_LAT)))


def local_to_map(odometry, forward, lateral):
    heading, position = yaw(odometry), odometry.pose.pose.position
    return position.x + forward * math.cos(heading) - lateral * math.sin(heading), position.y + forward * math.sin(heading) + lateral * math.cos(heading)


def polygon_at(odometry):
    # Keep the fixture finite and far enough ahead that the Dubins planner
    # (4 m minimum turning radius) has space to produce a genuine detour.
    # The zones manager adds the operational 1.5 m degradation halo, so a
    # closer or wider rectangle accidentally becomes an impassable wall.
    points = [local_to_map(odometry, forward, lateral) for forward, lateral in ((8.0, -0.5), (10.0, -0.5), (10.0, 0.5), (8.0, 0.5), (8.0, -0.5))]
    return {"type": "FeatureCollection", "features": [{"type": "Feature", "properties": {"id": "smoke_block", "enabled": True}, "geometry": {"type": "Polygon", "coordinates": [[[lon, lat] for lat, lon in (lat_lon(x, y) for x, y in points)]]}}]}


def goal_request(x, y, heading):
    request = SetNavGoalLL.Request(); request.lat, request.lon = lat_lon(x, y); request.yaw_deg = math.degrees(heading)
    return request


def projected_contains(message, x, y):
    def contains(ring):
        inside = False
        for index, point in enumerate(ring):
            previous = ring[index - 1]
            if ((point.y > y) != (previous.y > y)) and x < (previous.x - point.x) * (y - point.y) / (previous.y - point.y) + point.x:
                inside = not inside
        return inside
    return any(contains(polygon.outer.points) and not any(contains(hole.points) for hole in polygon.holes) for polygon in message.polygons)


def costmap_has_core(costmap, x, y):
    meta = costmap.metadata
    i, j = int((x - meta.origin.position.x) / meta.resolution), int((y - meta.origin.position.y) / meta.resolution)
    return 0 <= i < meta.size_x and 0 <= j < meta.size_y and costmap.data[j * meta.size_x + i] == 254


def main():
    rclpy.init(); node = ZonesSmoke()
    runtime = SmokeRuntime(
        node, "keepout-runtime",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "zones_probe.json",
    )
    node.runtime = runtime
    success = False
    failure = None
    try:
        runtime.wait(
            "zones startup readiness unavailable",
            node.startup_ready,
            45.0,
            stimulate=node.poll_bt_state,
            observe=node.startup_evidence,
        )
        initial_state = AsyncServicePoller(
            node.get_zones, GetZonesState.Request,
            interval_s=0.25, response_timeout_s=40.0,
        )
        runtime.wait(
            "zones manager initial mask ready",
            lambda: initial_state.latest is not None and initial_state.latest.mask_ready,
            45.0,
            stimulate=initial_state.poll,
            observe=lambda: {
                **initial_state.evidence(),
                "mask_ready": (
                    initial_state.latest.mask_ready
                    if initial_state.latest is not None else False
                ),
            },
        )
        wait_for(node, lambda: node.odom and node.projected, 20.0, "global odometry or projected keepouts unavailable")
        current = node.odom[-1]
        response = call(node, node.set_zones, SetZonesGeoJson.Request(geojson=json.dumps(polygon_at(current))), "set zones unavailable")
        if not response.ok or response.polygon_count != 1 or not response.map_reloaded: raise RuntimeError(response.error or "zone was not applied")
        wait_for(node, lambda: node.projected and node.projected[-1].polygons, 12.0, "projected keepout state has no polygons")
        state = call(node, node.get_zones, GetZonesState.Request(), "get zones unavailable")
        if not state.mask_ready or "smoke_block" not in state.geojson: raise RuntimeError("zone state was not persisted")
        blocked_x, blocked_y = local_to_map(current, 9.0, 0.0)
        wait_for(node, lambda: node.projected and projected_contains(node.projected[-1], blocked_x, blocked_y), 4.0, "projected keepout state does not cover the blocked goal")
        blocked = call(node, node.goal, goal_request(blocked_x, blocked_y, yaw(current)), "goal service unavailable")
        if blocked.ok: raise RuntimeError("goal inside keepout zone was accepted")
        wait_for(node, lambda: node.global_costmaps and costmap_has_core(node.global_costmaps[-1], blocked_x, blocked_y), 12.0, "vector keepout was not rasterized in planner costmap")
        node.plans.clear()
        destination_x, destination_y = local_to_map(current, 22.0, 0.0)
        accepted = call(node, node.goal, goal_request(destination_x, destination_y, yaw(current)), "goal service unavailable")
        if not accepted.ok: raise RuntimeError(accepted.error)
        wait_for(node, lambda: node.plans and len(node.plans[-1].poses) > 2, 12.0, "planner did not produce an avoidance plan")
        # A direct segment would remain at lateral coordinate zero; the plan must detour.
        start = current.pose.pose.position; heading = yaw(current)
        deviations = [abs(-(pose.pose.position.x - start.x) * math.sin(heading) + (pose.pose.position.y - start.y) * math.cos(heading)) for pose in node.plans[-1].poses]
        if max(deviations) < 1.0: raise RuntimeError("planner did not avoid the keepout zone")
        empty = call(node, node.set_zones, SetZonesGeoJson.Request(geojson=json.dumps({"type": "FeatureCollection", "features": []})), "set zones unavailable")
        if not empty.ok: raise RuntimeError(empty.error)
        persisted = call(node, node.reload, Trigger.Request(), "reload zones unavailable")
        if not persisted.success: raise RuntimeError(persisted.message)
        state = call(node, node.get_zones, GetZonesState.Request(), "get zones unavailable")
        if json.loads(state.geojson)["features"]: raise RuntimeError("empty persisted zone set did not reload")
        print("Navigation zones simulation smoke test passed")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "odometry": len(node.odom),
            "projected_keepout_states": len(node.projected),
            "plans": len(node.plans),
            "zones_startup": node.startup_evidence(),
        })
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
