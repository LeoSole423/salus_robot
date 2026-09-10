#!/usr/bin/env python3
"""Capture bounded clock and odometry sequences in the caller's ROS domain."""

from __future__ import annotations

import argparse
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.node import Node


def _stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clock-samples", type=int, default=5)
    parser.add_argument("--odometry-samples", type=int, default=3)
    parser.add_argument("--timeout-s", type=float, default=5.0)
    args = parser.parse_args(argv)
    if min(args.clock_samples, args.odometry_samples) < 1 or args.timeout_s <= 0:
        parser.error("sample counts and timeout must be positive")
    rclpy.init(args=None)
    node = Node("salus_navigation_isolation_probe")
    clocks = []
    odometry = []

    def on_clock(message):
        clocks.append(_stamp_ns(message.clock))

    def on_odometry(message):
        position = message.pose.pose.position
        values = (_stamp_ns(message.header.stamp), float(position.x), float(position.y))
        if all(math.isfinite(value) for value in values):
            odometry.append({"stamp_ns": int(values[0]), "x_m": values[1],
                             "y_m": values[2]})

    node.create_subscription(Clock, "/clock", on_clock, 10)
    node.create_subscription(Odometry, "/odometry/global", on_odometry, 10)
    deadline = time.monotonic() + args.timeout_s
    try:
        while time.monotonic() < deadline and (
            len(clocks) < args.clock_samples or len(odometry) < args.odometry_samples
        ):
            rclpy.spin_once(node, timeout_sec=min(.25, max(0.0, deadline - time.monotonic())))
        publisher_count = len(node.get_publishers_info_by_topic(
            "/clock", no_mangle=True
        ))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    payload = {
        "clock_ns": clocks,
        "odometry": odometry,
        "clock_complete": len(clocks) >= args.clock_samples,
        "odometry_complete": len(odometry) >= args.odometry_samples,
        "clock_publisher_count": publisher_count,
    }
    print(json.dumps(payload, sort_keys=True))
    return 0 if payload["clock_complete"] and payload["odometry_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
