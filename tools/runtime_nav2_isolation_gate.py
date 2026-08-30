#!/usr/bin/env python3
"""Native readiness gate for the #119 A/B Nav2 isolation experiment."""

from __future__ import annotations

import argparse
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from smoke_runtime import SmokeRuntime, subscribe_navigation_startup


class IsolationGate(Node):
    def __init__(self, require_nav2: bool) -> None:
        super().__init__(
            "runtime_nav2_isolation_gate",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.local_odom = 0
        self.global_odom = 0
        self.local_costmap = 0
        self.require_nav2 = require_nav2
        self.startup = subscribe_navigation_startup(self) if require_nav2 else None
        self.create_subscription(
            Odometry, "/odometry/local", lambda _msg: self._inc("local_odom"), 10
        )
        self.create_subscription(
            Odometry, "/odometry/global", lambda _msg: self._inc("global_odom"), 10
        )
        grid_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            lambda _msg: self._inc("local_costmap"),
            grid_qos,
        )

    def _inc(self, name: str) -> None:
        setattr(self, name, getattr(self, name) + 1)

    def ready(self) -> bool:
        base = self.local_odom > 0 and self.global_odom > 0
        if not self.require_nav2:
            return base
        return bool(
            base
            and self.local_costmap > 0
            and self.startup is not None
            and self.startup.is_ready()
        )

    def evidence(self) -> dict:
        return {
            "local_odom_messages": self.local_odom,
            "global_odom_messages": self.global_odom,
            "local_costmap_messages": self.local_costmap,
            "navigation_startup": (
                self.startup.snapshot() if self.startup is not None else None
            ),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--require-nav2", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=60.0)
    args = parser.parse_args()

    rclpy.init()
    node = IsolationGate(args.require_nav2)
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                print(node.evidence(), flush=True)
                return 0
        print(node.evidence(), flush=True)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
