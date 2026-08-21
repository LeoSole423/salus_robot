#!/usr/bin/env python3
"""Exercise the latched battery return through public patrol contracts."""
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from salus_interfaces.msg import BatteryMissionGuard, NavEvent
from salus_interfaces.srv import (
    CancelPatrolMission, GetPatrolMissionState, SetPatrolMissionLL,
)
from smoke_runtime import (
    AsyncServicePoller, SmokeRuntime, finite_odometry, has_increasing_stamps,
    subscribe_navigation_startup,
)


LAT, LON = -31.4858037, -64.2410570


class Smoke(Node):
    def __init__(self):
        super().__init__(
            "patrol_battery_smoke",
            parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom = []
        self.events = []
        self.guard_low = False
        self.guard_publications = 0
        self.last_guard_publish = 0.0
        self.phase_history = []
        self.create_subscription(Odometry, "/odometry/global", self.odom.append, 10)
        self.create_subscription(
            NavEvent, "/nav_command_server/events", self.events.append, 10)
        self.guard = self.create_publisher(
            BatteryMissionGuard, "/smoke/battery_mission_guard", 10)
        self.set_patrol = self.create_client(
            SetPatrolMissionLL, "/route_executor/set_patrol_mission_ll")
        self.state = self.create_client(
            GetPatrolMissionState, "/route_executor/get_patrol_mission_state")
        self.cancel = self.create_client(
            CancelPatrolMission, "/route_executor/cancel_patrol_mission")
        self.startup = subscribe_navigation_startup(self)

    def publish_guard(self):
        now = time.monotonic()
        if now - self.last_guard_publish < 0.1:
            return
        message = BatteryMissionGuard()
        message.stamp = self.get_clock().now().to_msg()
        message.ready = True
        message.fresh = True
        message.return_home_recommended = self.guard_low
        message.state = "RETURN_HOME" if self.guard_low else "WATCHING"
        message.operator_soc_pct = 20.0 if self.guard_low else 80.0
        self.guard.publish(message)
        self.guard_publications += 1
        self.last_guard_publish = now


def ll_from_local(x, y):
    return (
        LAT + y / 111_320.0,
        LON + x / (111_320.0 * math.cos(math.radians(LAT))),
    )


def patrol_request(pose, *, home_at_origin):
    yaw = math.atan2(
        2.0 * pose.orientation.w * pose.orientation.z,
        1.0 - 2.0 * pose.orientation.z ** 2)
    origin_x, origin_y = pose.position.x, pose.position.y

    def world(forward, left):
        return (
            origin_x + forward * math.cos(yaw) - left * math.sin(yaw),
            origin_y + forward * math.sin(yaw) + left * math.cos(yaw),
        )

    loop_xy = [world(3.0, 0.0), world(6.0, 0.0), world(9.0, 0.0)]
    loop_ll = [ll_from_local(x, y) for x, y in loop_xy]
    home_xy = (origin_x, origin_y) if home_at_origin else loop_xy[-1]
    home_lat, home_lon = ll_from_local(*home_xy)
    request = SetPatrolMissionLL.Request()
    request.loop_lats = [value[0] for value in loop_ll]
    request.loop_lons = [value[1] for value in loop_ll]
    request.loop_waypoint_action_jsons = ["", "", ""]
    request.home_lat, request.home_lon = home_lat, home_lon
    request.home_yaw_deg = math.degrees(yaw)
    request.depart_entry_loop_index = 0
    request.leg_spacing_m = 2.0
    request.chunk_span_m = 8.0
    request.chunk_max_waypoints = 4
    return request


def main():
    rclpy.init()
    node = Smoke()
    runtime = SmokeRuntime(
        node, "patrol-battery-return",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "patrol_battery_probe.json",
        global_timeout_s=220.0)
    success = False
    failure = None
    poller = AsyncServicePoller(
        node.state, GetPatrolMissionState.Request,
        interval_s=0.25, response_timeout_s=5.0)

    def stimulate():
        node.publish_guard()
        poller.poll()
        if poller.latest is not None:
            phase = poller.latest.status
            if not node.phase_history or node.phase_history[-1] != phase:
                node.phase_history.append(phase)

    def state_is(*phases):
        return poller.latest is not None and poller.latest.status in phases

    try:
        runtime.wait(
            "patrol startup", lambda: (
                node.startup.active
                and node.set_patrol.service_is_ready()
                and node.state.service_is_ready()
                and node.cancel.service_is_ready()
                and has_increasing_stamps(node.odom)
                and finite_odometry(node.odom[-1])
                and node.guard.get_subscription_count() >= 1),
            45.0, stimulate=stimulate,
            observe=lambda: {
                "startup": node.startup.snapshot(),
                "odom_messages": len(node.odom),
                "guard_subscribers": node.guard.get_subscription_count(),
                "services": [node.set_patrol.service_is_ready(),
                             node.state.service_is_ready(),
                             node.cancel.service_is_ready()],
            })

        # A mission accepted while the guard already requests return must stay
        # parked at HOME and must never dispatch its departure/loop.
        node.guard_low = True
        runtime.wait(
            "low guard delivered", lambda: node.guard_publications >= 3,
            5.0, stimulate=stimulate)
        request = patrol_request(node.odom[-1].pose.pose, home_at_origin=True)
        accepted = runtime.call(
            "set low-battery patrol", node.set_patrol, request, timeout_s=8.0)
        if not accepted.ok:
            raise RuntimeError(f"low-battery patrol rejected: {accepted.error}")
        runtime.wait(
            "patrol held at HOME",
            lambda: state_is("AT_HOME") and poller.latest.low_battery_active
            and not poller.latest.active,
            15.0, stimulate=stimulate,
            observe=lambda: None if poller.latest is None else {
                "status": poller.latest.status,
                "active": poller.latest.active,
                "low_battery_active": poller.latest.low_battery_active,
            })
        dispatched_before_cancel = sum(
            event.code == "PATROL_PHASE_DISPATCHED" for event in node.events)
        if dispatched_before_cancel:
            raise RuntimeError("low-battery patrol dispatched motion from HOME")
        cancelled = runtime.call(
            "cancel parked patrol", node.cancel,
            CancelPatrolMission.Request(), timeout_s=8.0)
        if not cancelled.ok:
            raise RuntimeError(f"parked patrol cancellation failed: {cancelled.error}")

        # Start normally, enter the loop, then latch a battery return. Recovery
        # samples remain false only after the return is already committed.
        node.guard_low = False
        baseline_publications = node.guard_publications
        runtime.wait(
            "healthy guard delivered",
            lambda: node.guard_publications >= baseline_publications + 3,
            5.0, stimulate=stimulate)
        accepted = runtime.call(
            "set active patrol", node.set_patrol,
            patrol_request(node.odom[-1].pose.pose, home_at_origin=False),
            timeout_s=8.0)
        if not accepted.ok:
            raise RuntimeError(f"active patrol rejected: {accepted.error}")
        runtime.wait(
            "patrol entered loop", lambda: state_is("PATROL"),
            70.0, stimulate=stimulate,
            observe=lambda: {"phase_history": node.phase_history})
        node.guard_low = True
        runtime.wait(
            "battery return requested",
            lambda: state_is("EXIT_LOOP", "RETURN_HOME", "AT_HOME")
            and poller.latest.low_battery_active,
            12.0, stimulate=stimulate,
            observe=lambda: {"phase_history": node.phase_history})
        node.guard_low = False
        runtime.wait(
            "latched return reached HOME",
            lambda: state_is("AT_HOME") and poller.latest.low_battery_active
            and not poller.latest.active,
            90.0, stimulate=stimulate,
            observe=lambda: {
                "phase_history": node.phase_history,
                "return_requested": (False if poller.latest is None
                                     else poller.latest.return_home_requested),
                "return_active": (False if poller.latest is None
                                  else poller.latest.return_home_active),
            })
        required = {"PATROL", "EXIT_LOOP", "RETURN_HOME", "AT_HOME"}
        if not required.issubset(set(node.phase_history)):
            raise RuntimeError(
                f"incomplete battery return phases: {node.phase_history}")
        success = True
        runtime.report.evidence = {
            "phase_history": node.phase_history,
            "guard_publications": node.guard_publications,
            "event_codes": [event.code for event in node.events],
            "state_poller": poller.evidence(),
        }
    except Exception as exc:
        failure = exc
        runtime.report.evidence = {
            "phase_history": node.phase_history,
            "guard_publications": node.guard_publications,
            "event_codes": [event.code for event in node.events[-20:]],
            "state_poller": poller.evidence(),
            "startup": node.startup.snapshot(),
        }
    finally:
        runtime.finish(success, error=failure)
        node.destroy_node()
        rclpy.shutdown()
    if failure is not None:
        print(f"Patrol battery smoke failed: {failure}", file=sys.stderr)
        return 1
    print("Patrol battery return smoke passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
