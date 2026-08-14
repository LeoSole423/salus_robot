#!/usr/bin/env python3
"""Bounded lifecycle readiness probe for ROS 2 smoke tests.

This deliberately avoids the ``ros2 lifecycle get`` CLI: a stalled CLI
request can prevent a shell-based readiness loop from reaching its deadline.
Service calls here are asynchronous and are driven until one monotonic-clock
deadline shared by every requested node.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


@dataclass
class LifecycleEvidence:
    node: str
    service: str
    attempts: int = 0
    service_available: bool = False
    response_received: bool = False
    first_response_latency_s: float | None = None
    last_state_id: int | None = None
    last_state_label: str | None = None
    next_attempt_at_s: float = 0.0
    failure: str = ""

    @property
    def active(self) -> bool:
        return self.last_state_id == State.PRIMARY_STATE_ACTIVE

    def record_state(self, state_id: int, label: str, latency_s: float) -> None:
        self.response_received = True
        if self.first_response_latency_s is None:
            self.first_response_latency_s = latency_s
        self.last_state_id = int(state_id)
        self.last_state_label = label

    def finish(self) -> None:
        if self.active:
            self.failure = ""
        elif not self.service_available:
            self.failure = "lifecycle service unavailable"
        elif not self.response_received:
            self.failure = "lifecycle service did not respond"
        else:
            self.failure = (
                f"lifecycle state is {self.last_state_label!r} "
                f"({self.last_state_id}), expected 'active'"
            )


class LifecycleReadinessProbe(Node):
    def __init__(self, node_names: list[str], started_at: float) -> None:
        super().__init__("lifecycle_readiness_probe")
        self.started_at = started_at
        self.lifecycle_clients = {}
        self.pending = {}
        self.evidence = {}
        for node_name in node_names:
            normalized = node_name.rstrip("/") or "/"
            service = f"{normalized}/get_state" if normalized != "/" else "/get_state"
            self.lifecycle_clients[normalized] = self.create_client(GetState, service)
            self.evidence[normalized] = LifecycleEvidence(node=normalized, service=service)

    def poll(self) -> None:
        now = time.monotonic()
        for node_name, client in self.lifecycle_clients.items():
            evidence = self.evidence[node_name]
            if evidence.active:
                continue
            if not client.service_is_ready():
                continue
            evidence.service_available = True
            future = self.pending.get(node_name)
            if future is None:
                if now < evidence.next_attempt_at_s:
                    continue
                evidence.attempts += 1
                evidence.next_attempt_at_s = now + 0.25
                self.pending[node_name] = client.call_async(GetState.Request())
                continue
            if not future.done():
                continue
            self.pending.pop(node_name, None)
            try:
                response = future.result()
            except Exception as exc:  # Service failures are reported, then retried until deadline.
                evidence.failure = f"lifecycle service error: {exc}"
                continue
            if response is None:
                evidence.failure = "lifecycle service returned no response"
                continue
            state = response.current_state
            evidence.record_state(state.id, state.label, time.monotonic() - self.started_at)

    def all_active(self) -> bool:
        return all(evidence.active for evidence in self.evidence.values())

    def report(self, timeout_s: float, success: bool) -> dict:
        for evidence in self.evidence.values():
            evidence.finish()
        return {
            "success": success,
            "timeout_s": timeout_s,
            "nodes": [asdict(evidence) | {"active": evidence.active} for evidence in self.evidence.values()],
        }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", action="append", required=True, help="Lifecycle node name; repeatable.")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument(
        "--report-path",
        type=Path,
        default=Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "lifecycle_probe.json",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0.0:
        raise ValueError("--timeout must be positive")
    started_at = time.monotonic()
    rclpy.init()
    probe = LifecycleReadinessProbe(args.node, started_at)
    success = False
    try:
        deadline = started_at + args.timeout
        while time.monotonic() < deadline:
            probe.poll()
            if probe.all_active():
                success = True
                break
            rclpy.spin_once(probe, timeout_sec=min(0.1, max(0.0, deadline - time.monotonic())))
        report = probe.report(args.timeout, success)
        args.report_path.parent.mkdir(parents=True, exist_ok=True)
        args.report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if success:
            print(f"Lifecycle readiness passed; report: {args.report_path}")
            return 0
        reasons = "; ".join(f"{item['node']}: {item['failure']}" for item in report["nodes"])
        raise RuntimeError(f"Lifecycle readiness timed out: {reasons}")
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
