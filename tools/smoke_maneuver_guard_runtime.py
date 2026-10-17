#!/usr/bin/env python3
"""Software-only ROS fault injection; no simulator, hardware or velocity owner.

Runs path health, the canonical arbiter and route executor in an isolated DDS
context. Only the input /cmd_vel_safe is stimulated; output has no controller.
"""
import json
import math
import time
from pathlib import Path
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path as NavPath
from salus_interfaces.msg import CmdVelFinal
from salus_interfaces.srv import (
    EvaluatePathHealth, GetRouteMissionState, SetManualMode, CancelRouteMission,
)
from salus_navigation.path_health import PathHealthNode
from salus_navigation.nav_command_server import NavCommandServer
from salus_navigation.route_executor_node import RouteExecutorNode
from salus_navigation.route_model import PreparedRoute, RouteMission, RoutePhase, RouteWaypoint
from salus_web.ros_gateway import _message_dict
from tf2_ros import TransformBroadcaster


def main():
    rclpy.init(args=['--ros-args', '-p', 'obstacle_detection_required:=false',
                    '-p', 'maneuver_wait_s:=2.0'])
    probe = Node('maneuver_fault_injection')
    health, arbiter, route = PathHealthNode(), NavCommandServer(), RouteExecutorNode()
    executor = MultiThreadedExecutor(num_threads=6)
    for node in (probe, health, arbiter, route):
        executor.add_node(node)
    tf = TransformBroadcaster(probe)
    costmap_pub = probe.create_publisher(Costmap, '/global_costmap/costmap_raw', 10)
    command_pub = probe.create_publisher(Twist, '/cmd_vel_safe', 10)
    outputs = []
    probe.create_subscription(CmdVelFinal, '/cmd_vel_final', outputs.append, 10)
    client = probe.create_client(EvaluatePathHealth, '/path_health/evaluate')
    state = probe.create_client(GetRouteMissionState, '/route_executor/get_route_mission_state')
    manual = probe.create_client(SetManualMode, '/nav_command_server/set_manual_mode')
    cancel = probe.create_client(CancelRouteMission, '/route_executor/cancel_route_mission')
    teleop = probe.create_publisher(CmdVelFinal, '/cmd_vel_teleop', 10)
    waypoints = tuple(RouteWaypoint(0., 0., 0., i, map_x=float(i * 20), map_y=0.) for i in range(2))
    route._mission = RouteMission(
        PreparedRoute(waypoints, False, 2, 35., 60., 80),
        RoutePhase.ACTIVE, mission_id='preserved-mission', target_index=1)

    def inputs():
        stamp = probe.get_clock().now().to_msg()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id, transform.child_frame_id = 'map', 'base_footprint'
        transform.transform.rotation.w = 1.
        tf.sendTransform(transform)
        costmap = Costmap()
        costmap.header.stamp, costmap.header.frame_id = stamp, 'map'
        costmap.metadata.resolution = 10.
        costmap.metadata.size_x = costmap.metadata.size_y = 20
        costmap.metadata.origin.position.x = costmap.metadata.origin.position.y = -100.
        costmap.data = [0] * 400
        costmap_pub.publish(costmap)
        command = Twist()
        command.linear.x = 0.5
        command_pub.publish(command)

    probe.create_timer(.05, inputs)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def until(predicate, timeout=8.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError('causal readiness/response timeout')

    def call(service, request):
        until(service.service_is_ready)
        future = service.call_async(request)
        until(future.done)
        return future.result()

    def evaluate(points, context=0):
        request = EvaluatePathHealth.Request(context=context)
        request.path = NavPath()
        request.path.header.frame_id = 'map'
        for x, y in points:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = float(x), float(y)
            request.path.poses.append(pose)
        return call(client, request).health

    def stopped():
        return outputs and outputs[-1].twist.linear.x == 0. and outputs[-1].brake_pct == 100

    try:
        until(lambda: bool(outputs) and health._costmap is not None)
        good = ((0., 0.), (20., 0.))
        until(lambda: evaluate(good).state == 0)
        until(lambda: outputs[-1].twist.linear.x > 0)
        u = tuple((4 * math.sin(t * math.pi / 90),
                   4 * (1 - math.cos(t * math.pi / 90))) for t in range(91))
        assert evaluate(u, 1).state == 2
        until(stopped)
        assert evaluate(good, 1).state == 0
        assert stopped(), 'candidate validation released old path'
        assert evaluate(good).state == 0
        until(lambda: outputs[-1].twist.linear.x > 0)
        assert evaluate(u, 1).state == 2
        for _ in range(3):
            assert evaluate(u, 1).state == 2
        until(lambda: route._recovery.state.value == 'NEEDS_OPERATOR')
        until(stopped)
        snapshot = call(state, GetRouteMissionState.Request())
        payload = _message_dict(snapshot)
        assert payload['blocked_state'] == 'NEEDS_OPERATOR'
        assert payload['blocked_reason_code'] == 'PLANNER_U_TURN'
        assert 'giro en U' in payload['blocked_reason_text']
        assert payload['mission_id'] == 'preserved-mission'
        assert payload['current_start_index'] == 1
        assert payload['reached_checkpoint_count'] == 0
        assert evaluate(good, 1).state == 2
        assert stopped()
        assert call(manual, SetManualMode.Request(enabled=True)).ok
        manual_command = CmdVelFinal()
        manual_command.twist.linear.x = .2

        def manual_seen():
            teleop.publish(manual_command)
            return outputs[-1].source == CmdVelFinal.SOURCE_MANUAL and outputs[-1].twist.linear.x > 0

        until(manual_seen)
        assert call(manual, SetManualMode.Request(enabled=False)).ok
        until(stopped)
        assert call(cancel, CancelRouteMission.Request()).ok
        cancelled = call(state, GetRouteMissionState.Request())
        assert cancelled.status == 'CANCELLED' and cancelled.reached_checkpoint_count == 0
        assert evaluate(good, 1).state == 2  # Cancel does not authorize an old path.
        output = Path('artifacts/maneuver-guard-runtime.json')
        output.parent.mkdir(exist_ok=True)
        output.write_text(json.dumps({'passed': True, 'route_mission': payload,
                                     'output_samples': len(outputs), 'manual_and_cancel_precedence': True}, indent=2))
        print('maneuver_runtime: PASS (stop, adoption, total timeout, mission, Cockpit payload)')
    finally:
        executor.shutdown(timeout_sec=3.)
        thread.join(timeout=3.)
        for node in (route, arbiter, health, probe):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
