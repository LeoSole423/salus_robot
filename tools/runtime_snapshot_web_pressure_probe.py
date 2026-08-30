#!/usr/bin/env python3
"""Controlled Snapshot/Web pressure probe for #119 Phase 2."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import os
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from salus_interfaces.srv import GetNavSnapshot
from sensor_msgs.msg import LaserScan

from smoke_runtime import SmokeRuntime, subscribe_navigation_startup


def _spin_for(node: Node, duration_s: float) -> None:
    deadline = time.monotonic() + max(0.0, duration_s)
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))


class PressureProbe(Node):
    def __init__(self) -> None:
        super().__init__(
            "runtime_snapshot_pressure_probe",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.startup = subscribe_navigation_startup(self)
        self.local_odom: Odometry | None = None
        self.fixture_scans_published = 0
        self.create_subscription(Odometry, "/odometry/local", self._on_odom, 10)
        self.scan = self.create_publisher(
            LaserScan, "/scan_clean", qos_profile_sensor_data
        )
        self.snapshot = self.create_client(
            GetNavSnapshot, "/nav_snapshot_server/get_nav_snapshot"
        )

    def _on_odom(self, message: Odometry) -> None:
        self.local_odom = message

    def publish_fixture_scan(self) -> None:
        if self.local_odom is None:
            return
        message = LaserScan()
        message.header.frame_id = "base_footprint"
        message.header.stamp = self.local_odom.header.stamp
        message.angle_min = -0.5
        message.angle_max = 0.5
        message.angle_increment = 0.1
        message.range_min = 0.4
        message.range_max = 20.0
        message.ranges = [math.inf] * 11
        message.ranges[5] = 4.0
        self.scan.publish(message)
        self.fixture_scans_published += 1


def _response_record(index: int, started: float, response=None, error: str = "") -> dict:
    completed = time.monotonic()
    return {
        "request_index": index,
        "started_monotonic_s": started,
        "completed_monotonic_s": completed,
        "duration_s": max(0.0, completed - started),
        "outcome": "response" if response is not None else "timeout",
        "response_ok": bool(response.ok) if response is not None else None,
        "response_error": str(response.error) if response is not None else error,
        "image_bytes": len(response.image_png) if response is not None else 0,
    }


def run_direct(node: PressureProbe, count: int, interval_s: float) -> list[dict]:
    deadline = time.monotonic() + 20.0
    while not node.snapshot.service_is_ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not node.snapshot.service_is_ready():
        raise RuntimeError("Snapshot service did not become ready")

    records: list[dict] = []
    for index in range(1, count + 1):
        _spin_for(node, 0.2)
        node.publish_fixture_scan()
        _spin_for(node, 0.1)
        started = time.monotonic()
        future = node.snapshot.call_async(GetNavSnapshot.Request())
        request_deadline = started + 20.0
        while not future.done() and time.monotonic() < request_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if future.done():
            try:
                response = future.result()
            except Exception as exc:
                records.append(
                    _response_record(index, started, error=f"{type(exc).__name__}: {exc}")
                )
            else:
                records.append(_response_record(index, started, response=response))
        else:
            future.cancel()
            records.append(_response_record(index, started, error="service timeout"))
        _spin_for(node, interval_s)
    return records


async def _connect_websocket(uri: str):
    import websockets

    socket = await websockets.connect(uri)
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline:
        incoming = json.loads(
            await asyncio.wait_for(socket.recv(), deadline - time.monotonic())
        )
        if incoming.get("op") == "state":
            return socket
    await socket.close()
    raise RuntimeError("WebSocket initial state did not arrive")


async def _run_web_requests(
    node: PressureProbe, uri: str, count: int, interval_s: float
) -> list[dict]:
    socket = await _connect_websocket(uri)
    records: list[dict] = []
    try:
        for index in range(1, count + 1):
            _spin_for(node, 0.2)
            node.publish_fixture_scan()
            _spin_for(node, 0.1)
            request_id = f"phase2-{index}"
            started = time.monotonic()
            await socket.send(json.dumps({
                "op": "get_nav_snapshot",
                "client_req_id": request_id,
            }))
            deadline = started + 25.0
            response = None
            error = ""
            try:
                while time.monotonic() < deadline:
                    incoming = json.loads(
                        await asyncio.wait_for(
                            socket.recv(), deadline - time.monotonic()
                        )
                    )
                    if incoming.get("client_req_id") == request_id:
                        response = incoming
                        break
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"

            completed = time.monotonic()
            records.append({
                "request_index": index,
                "started_monotonic_s": started,
                "completed_monotonic_s": completed,
                "duration_s": max(0.0, completed - started),
                "outcome": "response" if response is not None else "timeout",
                "response_ok": response.get("ok") if response is not None else None,
                "response_error": (
                    str(response.get("error", "")) if response is not None else error
                ),
                "image_b64_chars": (
                    len(str(response.get("image_b64", "")))
                    if response is not None else 0
                ),
            })
            _spin_for(node, interval_s)
    finally:
        await socket.close()
    return records


def run_web(node: PressureProbe, port: int, count: int, interval_s: float) -> list[dict]:
    return asyncio.run(
        _run_web_requests(
            node, f"ws://127.0.0.1:{port}", count, interval_s
        )
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("none", "direct", "web"), required=True)
    parser.add_argument("--requests", type=int, default=5)
    parser.add_argument("--interval-s", type=float, default=0.5)
    parser.add_argument("--web-port", type=int, default=18768)
    args = parser.parse_args()

    rclpy.init()
    node = PressureProbe()
    report_path = Path(
        os.environ.get("SMOKE_ARTIFACT_DIR", ".")
    ) / "snapshot_web_pressure.json"
    runtime = SmokeRuntime(
        node,
        "runtime-snapshot-web-pressure",
        report_path,
        global_timeout_s=180.0,
    )
    success = False
    failure = None
    records: list[dict] = []
    started = time.monotonic()
    try:
        runtime.wait_navigation_startup(node.startup, 60.0)
        runtime.wait(
            "local odometry for pressure fixture",
            lambda: node.local_odom is not None,
            10.0,
        )

        if args.mode == "none":
            _spin_for(node, 15.0)
        elif args.mode == "direct":
            records = run_direct(node, args.requests, args.interval_s)
        else:
            records = run_web(
                node, args.web_port, args.requests, args.interval_s
            )

        if args.mode != "none" and not any(
            record["outcome"] == "response" for record in records
        ):
            raise RuntimeError("pressure path produced no completed responses")

        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(
            success,
            error=failure,
            evidence={
                "mode": args.mode,
                "requests_target": args.requests,
                "fixture_scans_published": node.fixture_scans_published,
                "pressure_started_monotonic_s": started,
                "pressure_completed_monotonic_s": time.monotonic(),
                "requests": records,
                "navigation_startup": node.startup.snapshot(),
            },
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
