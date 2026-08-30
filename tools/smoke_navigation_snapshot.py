#!/usr/bin/env python3
"""Verify the public navigation snapshot service against the composed sim."""

import math
import os
import sys
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from salus_interfaces.srv import GetNavSnapshot
from sensor_msgs.msg import LaserScan

from smoke_runtime import AsyncServicePoller, SmokeRuntime, subscribe_navigation_startup


class SnapshotSmoke(Node):
    def __init__(self) -> None:
        super().__init__(
            "navigation_snapshot_smoke",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.client = self.create_client(
            GetNavSnapshot, "/nav_snapshot_server/get_nav_snapshot"
        )
        self.startup = subscribe_navigation_startup(self)
        self.local_odom: Odometry | None = None
        self.fixture_scans_published = 0
        self.fixture_scan_interval_s = 0.5
        self.next_fixture_scan_at = 0.0
        self.create_subscription(
            Odometry, "/odometry/local", self._on_local_odom, 10
        )
        self.scan = self.create_publisher(
            LaserScan, "/scan_clean", qos_profile_sensor_data
        )

    def _on_local_odom(self, message: Odometry) -> None:
        self.local_odom = message

    def publish_fixture_scan(self) -> None:
        """Publish a bounded-rate snapshot-only scan at a transformable stamp."""
        now = time.monotonic()
        if self.local_odom is None or now < self.next_fixture_scan_at:
            return
        self.next_fixture_scan_at = now + self.fixture_scan_interval_s
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


def main() -> int:
    rclpy.init()
    node = SnapshotSmoke()
    runtime = SmokeRuntime(
        node,
        "navigation-snapshot",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "snapshot_probe.json",
        global_timeout_s=120.0,
    )
    success, failure = False, None
    response = None
    try:
        runtime.wait(
            "navigation startup",
            lambda: node.startup.active,
            60.0,
            observe=node.startup.snapshot,
        )
        runtime.wait(
            "local odometry for scan fixture",
            lambda: node.local_odom is not None,
            10.0,
        )
        poller = AsyncServicePoller(
            node.client,
            GetNavSnapshot.Request,
            interval_s=0.5,
            response_timeout_s=20.0,
        )

        def stimulate_snapshot() -> None:
            node.publish_fixture_scan()
            poller.poll()

        runtime.wait(
            "snapshot readiness",
            lambda: bool(
                poller.latest
                and poller.latest.ok
                and poller.latest.layers.local_costmap
                and poller.latest.layers.global_costmap
                and poller.latest.layers.global_inset
                and poller.latest.layers.scan
            ),
            45.0,
            stimulate=stimulate_snapshot,
            observe=lambda: {
                **poller.evidence(),
                "fixture_scans_published": node.fixture_scans_published,
                "last_error": poller.latest.error if poller.latest else "",
            },
        )
        response = poller.latest
        if response is None:
            raise RuntimeError("snapshot service returned no response")
        if (
            response.mime != "image/png"
            or response.width != response.height
            or response.width < 128
        ):
            raise RuntimeError("snapshot metadata is invalid")
        if not bytes(response.image_png).startswith(b"\x89PNG\r\n\x1a\n"):
            raise RuntimeError("snapshot response is not a PNG")
        if not response.layers.local_costmap:
            raise RuntimeError("snapshot omitted required local costmap")
        if not response.layers.global_costmap or not response.layers.global_inset:
            raise RuntimeError("snapshot omitted global costmap inset")
        if not response.layers.scan:
            raise RuntimeError("snapshot omitted explicit /scan_clean fixture")
        png_path = (
            Path(os.environ.get("SMOKE_ARTIFACT_DIR", "."))
            / "navigation_snapshot.png"
        )
        png_path.write_bytes(bytes(response.image_png))
        print("Navigation snapshot simulation smoke test passed")
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
                "navigation_startup": node.startup.snapshot(),
                "fixture_scans_published": node.fixture_scans_published,
                "service_ready": node.client.service_is_ready(),
                "response_ok": bool(response and response.ok),
                "layers": {
                    name: bool(getattr(response.layers, name))
                    for name in (
                        "local_costmap",
                        "global_costmap",
                        "keepout_mask",
                        "footprint",
                        "stop_zone",
                        "scan",
                        "plan",
                        "collision_polygons",
                        "global_inset",
                    )
                }
                if response
                else {},
                "png_bytes": len(response.image_png) if response else 0,
            },
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
