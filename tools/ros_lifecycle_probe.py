#!/usr/bin/env python3
"""Read one lifecycle state directly, without the ros2 CLI daemon."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("node")
    parser.add_argument("--timeout-s", type=float, default=5.0)
    args = parser.parse_args(argv)
    rclpy.init(args=None)
    node = Node("salus_navigation_lifecycle_probe")
    client = node.create_client(GetState, f"{args.node.rstrip('/')}/get_state")
    state = None
    try:
        deadline = time.monotonic() + args.timeout_s
        while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=.25):
            pass
        if client.service_is_ready():
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=max(
                0.0, deadline - time.monotonic()
            ))
            if future.done() and future.exception() is None:
                state = future.result().current_state
    finally:
        node.destroy_node()
        rclpy.shutdown()
    payload = {"node": args.node, "state_id": (
        None if state is None else int(state.id)
    ), "state": None if state is None else state.label}
    print(json.dumps(payload, sort_keys=True))
    return 0 if state is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
