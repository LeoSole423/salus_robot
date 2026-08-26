#!/usr/bin/env python3
"""Causal control and simulated-battery contract smoke."""

import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import Twist
from salus_interfaces.msg import BatteryMissionGuard, CmdVelFinal, VehicleCommand
from salus_interfaces.srv import SetSimBatteryPreset, SetSimBatteryState
from smoke_runtime import SmokeRuntime, has_increasing_stamps


def guard_progresses(messages) -> bool:
    if len(messages) < 2:
        return False
    stamps = [item.stamp.sec * 1_000_000_000 + item.stamp.nanosec for item in messages[-2:]]
    return stamps[1] > stamps[0]


class ControlProbe(Node):
    def __init__(self):
        super().__init__("control_sim_smoke")
        self.battery = []
        self.guard = []
        self.vehicle_commands = []
        self.create_subscription(BatteryState, "/battery_state", self.battery.append, 10)
        self.create_subscription(BatteryMissionGuard, "/battery_mission_guard", self.guard.append, 10)
        self.create_subscription(
            VehicleCommand,
            "/vehicle/command_shadow",
            self.vehicle_commands.append,
            10,
        )
        self.command = self.create_publisher(CmdVelFinal, "/cmd_vel_final", 10)
        self.preset = self.create_client(SetSimBatteryPreset, "/sim_battery/set_preset")
        self.state = self.create_client(SetSimBatteryState, "/sim_battery/set_state")


def main() -> int:
    rclpy.init()
    node = ControlProbe()
    runtime = SmokeRuntime(
        node, "control-battery",
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "control_probe.json",
    )
    success = False
    failure = None
    applied = []
    try:
        runtime.wait_topic_publishers("battery publisher", "/battery_state")
        runtime.wait_topic_publishers("guard publisher", "/battery_mission_guard")
        runtime.wait_topic_publishers(
            "vehicle command shadow publisher", "/vehicle/command_shadow"
        )
        runtime.wait(
            "progressive battery publications",
            lambda: has_increasing_stamps(node.battery) and guard_progresses(node.guard),
            15.0,
            observe=lambda: {"battery": len(node.battery), "guard": len(node.guard)},
        )
        command = CmdVelFinal()
        command.twist = Twist()
        command.twist.linear.x = 2.0
        command.twist.angular.z = 0.4
        command.source = VehicleCommand.SOURCE_AUTO
        runtime.wait(
            "canonical vehicle command shadow",
            lambda: (
                node.command.publish(command) is None
                and any(
                    item.source == VehicleCommand.SOURCE_AUTO
                    and item.drive_enabled
                    and not item.emergency_stop
                    and abs(item.drive.speed - 2.0) < 1.0e-6
                    and item.valid_for.sec == 0
                    and item.valid_for.nanosec == 700_000_000
                    for item in node.vehicle_commands
                )
            ),
            10.0,
            observe=lambda: {"vehicle_commands": len(node.vehicle_commands)},
        )
        runtime.wait("battery state service", node.state.service_is_ready, 10.0)
        for preset in ("full", "under_load", "watching", "return_home_rest",
                       "return_home_load", "stale", "suspect", "unavailable"):
            response = runtime.call(
                f"preset {preset}", node.preset,
                SetSimBatteryPreset.Request(preset=preset), timeout_s=10.0,
            )
            if not response.ok or response.applied_preset != preset:
                raise RuntimeError(response.error or f"preset {preset!r} was not applied")
            applied.append(preset)
        success = True
        print("Control simulation smoke test passed")
        return 0
    except Exception as exc:
        failure = exc
        raise
    finally:
        runtime.finish(success, error=failure, evidence={
            "applied_presets": applied,
            "battery_messages": len(node.battery),
            "guard_messages": len(node.guard),
            "vehicle_command_messages": len(node.vehicle_commands),
        })
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
