#!/usr/bin/env python3
"""Set or read one ROS parameter without using the ros2 daemon."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node


def _wait_for(client, deadline):
    while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=.25):
        pass
    return client.service_is_ready()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("get", "set"))
    parser.add_argument("node")
    parser.add_argument("parameter")
    parser.add_argument("value", nargs="?", type=float)
    parser.add_argument("--timeout-s", type=float, default=5.0)
    args = parser.parse_args(argv)
    if args.action == "set" and args.value is None:
        parser.error("set requires a numeric value")

    rclpy.init(args=None)
    node = Node("salus_navigation_parameter_probe")
    deadline = time.monotonic() + args.timeout_s
    try:
        if args.action == "set":
            client = node.create_client(SetParameters,
                                        f"{args.node.rstrip('/')}/set_parameters")
            if not _wait_for(client, deadline):
                return 1
            value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                   double_value=args.value)
            request = SetParameters.Request(parameters=[Parameter(
                name=args.parameter, value=value)])
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=max(0.0, deadline - time.monotonic())
            )
            if not future.done() or future.exception() is not None:
                return 1
            result = future.result().results[0]
            print(json.dumps({"successful": result.successful,
                              "reason": result.reason}, sort_keys=True))
            return 0 if result.successful else 1

        client = node.create_client(GetParameters,
                                    f"{args.node.rstrip('/')}/get_parameters")
        if not _wait_for(client, deadline):
            return 1
        future = client.call_async(GetParameters.Request(names=[args.parameter]))
        rclpy.spin_until_future_complete(
            node, future, timeout_sec=max(0.0, deadline - time.monotonic())
        )
        if not future.done() or future.exception() is not None:
            return 1
        value = future.result().values[0]
        if value.type == ParameterType.PARAMETER_DOUBLE:
            number = value.double_value
        elif value.type == ParameterType.PARAMETER_INTEGER:
            number = value.integer_value
        elif value.type == ParameterType.PARAMETER_BOOL:
            print(f"Bool value is: {str(value.bool_value).lower()}")
            return 0
        else:
            return 1
        print(f"Double value is: {number}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
