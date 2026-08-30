"""Executable composition of the ROS gateway and WebSocket transport."""

from __future__ import annotations

import asyncio
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from salus_interfaces.srv import CameraPtzState

from .operator_guard import OperatorControlGuard
from .operator_lease import OperatorLease
from .ros_gateway import CockpitRosGateway
from .websocket_server import CockpitWebSocketServer


async def _serve(node: CockpitRosGateway) -> None:
    guard = OperatorControlGuard(
        enabled=bool(node.get_parameter("enable_control_lock").value),
        heartbeat_timeout_s=float(
            node.get_parameter("control_lock_heartbeat_timeout_s").value
        ),
        initially_locked=bool(
            node.get_parameter("control_lock_start_locked").value
        ),
        clock=time.monotonic,
    )
    server = CockpitWebSocketServer(
        node,
        OperatorLease(guard),
        host=str(node.get_parameter("ws_host").value),
        port=int(node.get_parameter("ws_port").value),
        queue_capacity=int(node.get_parameter("client_queue_capacity").value),
    )
    node.set_broadcast_callback(server.broadcast_from_thread)
    if bool(node.get_parameter("require_camera_service").value):
        await node.wait_for_required_service(
            "get_camera_ptz_state",
            CameraPtzState.Request(),
        )
    await server.start()
    host = node.get_parameter("ws_host").value
    port = node.get_parameter("ws_port").value
    node.get_logger().info(f"Cockpit WebSocket listening on {host}:{port}")
    try:
        while rclpy.ok():
            await asyncio.sleep(0.25)
    finally:
        await server.stop()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CockpitRosGateway()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        asyncio.run(_serve(node))
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)
