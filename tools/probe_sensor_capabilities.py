#!/usr/bin/env python3
"""Verify selected sensor capabilities become READY without changing source IDs."""

from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from salus_interfaces.msg import CapabilityState, SystemCapabilities


def capability_state_label(value: int) -> str:
    mapping = {
        CapabilityState.STATE_UNKNOWN: "unknown",
        CapabilityState.STATE_DISABLED_BY_PROFILE: "disabled_by_profile",
        CapabilityState.STATE_ENABLED_BY_PROFILE: "enabled_by_profile",
        CapabilityState.STATE_UNAVAILABLE: "unavailable",
        CapabilityState.STATE_READY: "ready",
        CapabilityState.STATE_STALE: "stale",
        CapabilityState.STATE_DEGRADED: "degraded",
    }
    return mapping.get(int(value), f"state_{int(value)}")


def capability_snapshot(message: SystemCapabilities | None) -> dict:
    if message is None:
        return {"message_received": False, "profile": "", "capabilities": {}}
    selected = {}
    for item in message.capabilities:
        if item.capability_id not in ("local_motion_imu", "global_orientation"):
            continue
        selected[item.capability_id] = {
            "state": int(item.state),
            "state_label": capability_state_label(item.state),
            "required": bool(item.required),
            "enabled": bool(item.enabled),
            "source_ids": list(item.source_ids),
            "detail": str(item.detail),
        }
    return {
        "message_received": True,
        "profile": str(message.profile),
        "capabilities": selected,
    }


class CapabilityProbe(Node):
    def __init__(self) -> None:
        super().__init__("sensor_capability_probe")
        self.latest = None
        self.messages = 0
        self.first_received_s = None
        self.last_received_s = None
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            SystemCapabilities, "/system/capabilities", self._on_capabilities, qos
        )

    def _on_capabilities(self, message: SystemCapabilities) -> None:
        now = time.monotonic()
        self.messages += 1
        if self.first_received_s is None:
            self.first_received_s = now
        self.last_received_s = now
        self.latest = message

    def evidence(self) -> dict:
        snapshot = capability_snapshot(self.latest)
        snapshot.update({
            "messages": self.messages,
            "first_received_s": self.first_received_s,
            "last_received_s": self.last_received_s,
        })
        return snapshot


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
                    raise RuntimeError(
                        "unexpected selected IMU source: "
                        + json.dumps(node.evidence(), sort_keys=True)
                    )
                if list(orientation.source_ids) != ["external_heading"]:
                    raise RuntimeError(
                        "unexpected selected orientation source: "
                        + json.dumps(node.evidence(), sort_keys=True)
                    )
                print("Selected sensor capabilities reached READY")
                return 0
        raise RuntimeError(
            "selected sensor capabilities did not reach READY: "
            + json.dumps(node.evidence(), sort_keys=True)
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
