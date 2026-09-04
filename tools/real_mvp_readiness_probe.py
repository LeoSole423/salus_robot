#!/usr/bin/env python3
"""Wait causally for the real MVP Nav2 startup diagnostic."""

from __future__ import annotations

import sys
import time
from typing import Iterable, Protocol


ACTIVE_MESSAGE = "ACTIVE: ALL_NAV2_NODES_ACTIVE"


class DiagnosticStatusLike(Protocol):
    message: str


def has_active_startup(statuses: Iterable[DiagnosticStatusLike]) -> bool:
    """Return whether a diagnostics sample reports the required Nav2 state."""

    return any(status.message == ACTIVE_MESSAGE for status in statuses)


def wait_for_active_startup(timeout_s: float) -> bool:
    """Subscribe once and wait only for the required diagnostics transition."""

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray

    active = False

    def on_diagnostics(message: DiagnosticArray) -> None:
        nonlocal active
        active = has_active_startup(message.status)

    rclpy.init(args=None)
    node = rclpy.create_node("real_mvp_readiness_probe")
    try:
        node.create_subscription(
            DiagnosticArray,
            "/navigation_startup/diagnostics",
            on_diagnostics,
            10,
        )
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not active:
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(remaining_s, 0.25))
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return active


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print("usage: real_mvp_readiness_probe.py TIMEOUT_S", file=sys.stderr)
        return 2

    try:
        timeout_s = float(argv[1])
    except ValueError:
        print("TIMEOUT_S must be numeric", file=sys.stderr)
        return 2
    if timeout_s <= 0.0:
        print("TIMEOUT_S must be positive", file=sys.stderr)
        return 2

    if wait_for_active_startup(timeout_s):
        print("REAL_MVP_READINESS_ACTIVE")
        return 0

    print("navigation startup did not become ACTIVE", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
