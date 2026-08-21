#!/usr/bin/env python3
"""Verify the public navigation snapshot service against the composed sim."""

import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from salus_interfaces.srv import GetNavSnapshot

from smoke_runtime import AsyncServicePoller, SmokeRuntime


class SnapshotSmoke(Node):
    def __init__(self) -> None:
        super().__init__("navigation_snapshot_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.client = self.create_client(GetNavSnapshot, "/nav_snapshot_server/get_nav_snapshot")


def main() -> int:
    rclpy.init()
    node = SnapshotSmoke()
    runtime = SmokeRuntime(node, "navigation-snapshot", Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "snapshot_probe.json")
    success, failure = False, None
    response = None
    try:
        poller = AsyncServicePoller(node.client, GetNavSnapshot.Request, interval_s=0.5, response_timeout_s=5.0)
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
            stimulate=poller.poll,
            observe=lambda: {**poller.evidence(), "last_error": poller.latest.error if poller.latest else ""},
        )
        response = poller.latest
        if response is None:
            raise RuntimeError("snapshot service returned no response")
        if response.mime != "image/png" or response.width != response.height or response.width < 128:
            raise RuntimeError("snapshot metadata is invalid")
        if not bytes(response.image_png).startswith(b"\x89PNG\r\n\x1a\n"):
            raise RuntimeError("snapshot response is not a PNG")
        if not response.layers.local_costmap:
            raise RuntimeError("snapshot omitted required local costmap")
        if not response.layers.global_costmap or not response.layers.global_inset:
            raise RuntimeError("snapshot omitted global costmap inset")
        if not response.layers.scan:
            raise RuntimeError("snapshot omitted fresh /scan_clean detections")
        png_path = Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "navigation_snapshot.png"
        png_path.write_bytes(bytes(response.image_png))
        print("Navigation snapshot simulation smoke test passed")
        success = True
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "service_ready": node.client.service_is_ready(),
            "response_ok": bool(response and response.ok),
            "layers": {name: bool(getattr(response.layers, name)) for name in (
                "local_costmap", "global_costmap", "keepout_mask", "footprint", "stop_zone",
                "scan", "plan", "collision_polygons", "global_inset")} if response else {},
            "png_bytes": len(response.image_png) if response else 0,
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
