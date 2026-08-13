#!/usr/bin/env python3
"""Exercise dynamic GeoJSON keepout masks against the active Nav2 stack."""

from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from salus_interfaces.srv import GetZonesState, SetNavGoalLL, SetZonesGeoJson
from std_srvs.srv import Trigger


DATUM_LAT, DATUM_LON = -31.4858037, -64.2410570


class ZonesSmoke(Node):
    def __init__(self):
        super().__init__("zones_sim_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom: list[Odometry] = []; self.mask: list[OccupancyGrid] = []; self.plans: list[Path] = []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        # map_server keeps this map latched. A late smoke subscriber must use
        # the matching QoS to receive the currently active empty/full mask.
        mask_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, "/keepout_filter_mask", self.mask.append, mask_qos
        )
        self.create_subscription(Path, "/plan", self.plans.append, 10)
        self.set_zones = self.create_client(SetZonesGeoJson, "/zones_manager/set_geojson")
        self.get_zones = self.create_client(GetZonesState, "/zones_manager/get_state")
        self.reload = self.create_client(Trigger, "/zones_manager/reload_from_disk")
        self.goal = self.create_client(SetNavGoalLL, "/nav_command_server/set_goal_ll")


def wait_for(node, predicate, timeout, message):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate(): return
    raise RuntimeError(message)


def call(node, client, request, message, timeout_s=20.0):
    if not client.wait_for_service(timeout_sec=10.0): raise RuntimeError(message)
    # Rendering and atomically loading the 3000x3000 keepout mask can exceed
    # the old eight-second bound on shared CI runners.
    future = client.call_async(request); wait_for(node, future.done, timeout_s, message)
    return future.result()


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


def mask_contains(mask, x, y):
    resolution = mask.info.resolution
    col = math.floor((x - mask.info.origin.position.x) / resolution)
    row = math.floor((y - mask.info.origin.position.y) / resolution)
    return 0 <= col < mask.info.width and 0 <= row < mask.info.height and mask.data[row * mask.info.width + col] >= 100


def main():
    rclpy.init(); node = ZonesSmoke()
    try:
        wait_for(node, lambda: node.odom and node.mask, 45.0, "global odometry or keepout mask unavailable")
        current = node.odom[-1]
        response = call(node, node.set_zones, SetZonesGeoJson.Request(geojson=json.dumps(polygon_at(current))), "set zones unavailable")
        if not response.ok or response.polygon_count != 1 or not response.map_reloaded: raise RuntimeError(response.error or "zone was not applied")
        wait_for(node, lambda: any(any(value >= 100 for value in item.data) for item in node.mask), 12.0, "keepout mask has no occupied cells")
        state = call(node, node.get_zones, GetZonesState.Request(), "get zones unavailable")
        if not state.mask_ready or "smoke_block" not in state.geojson: raise RuntimeError("zone state was not persisted")
        blocked_x, blocked_y = local_to_map(current, 9.0, 0.0)
        wait_for(node, lambda: node.mask and mask_contains(node.mask[-1], blocked_x, blocked_y), 4.0, "updated keepout mask does not cover the blocked goal")
        blocked = call(node, node.goal, goal_request(blocked_x, blocked_y, yaw(current)), "goal service unavailable")
        if blocked.ok: raise RuntimeError("goal inside keepout zone was accepted")
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
        return 0
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
