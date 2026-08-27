#!/usr/bin/env python3
"""Verify selected sensor capabilities become READY without changing source IDs."""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from salus_interfaces.msg import CapabilityState, SystemCapabilities


class CapabilityProbe(Node):
    def __init__(self) -> None:
        super().__init__("sensor_capability_probe")
        self.latest = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            SystemCapabilities, "/system/capabilities", self._on_capabilities, qos
        )

    def _on_capabilities(self, message: SystemCapabilities) -> None:
        self.latest = message


def main() -> int:
    rclpy.init()
    node = CapabilityProbe()
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.latest is None:
                continue
            capabilities = {
                item.capability_id: item for item in node.latest.capabilities
            }
            imu = capabilities.get("local_motion_imu")
            orientation = capabilities.get("global_orientation")
            if (
                imu is not None
                and orientation is not None
                and imu.state == CapabilityState.STATE_READY
                and orientation.state == CapabilityState.STATE_READY
            ):
                if list(imu.source_ids) != ["imu_primary"]:
                    raise RuntimeError("unexpected selected IMU source")
                if list(orientation.source_ids) != ["external_heading"]:
                    raise RuntimeError("unexpected selected orientation source")
                print("Selected sensor capabilities reached READY")
                return 0
        raise RuntimeError("selected sensor capabilities did not reach READY")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
