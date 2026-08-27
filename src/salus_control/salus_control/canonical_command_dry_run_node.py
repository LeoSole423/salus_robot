"""Validate canonical commands with no actuation side effects."""

from __future__ import annotations

import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from salus_interfaces.msg import VehicleCommand

from .canonical_command_consumer import (
    CanonicalCommandConfig,
    CanonicalCommandConsumer,
    CanonicalCommandSample,
)


class CanonicalCommandDryRunNode(Node):
    """Exercise validation/watchdog policy while publishing diagnostics only."""

    def __init__(self, *, parameter_overrides=None, monotonic_clock=time.monotonic):
        super().__init__(
            "canonical_command_dry_run", parameter_overrides=parameter_overrides
        )
        defaults = {
            "input_topic": "/vehicle/command_shadow",
            "diagnostics_topic": "/vehicle/command_dry_run/diagnostics",
            "max_forward_speed_mps": 4.0,
            "max_reverse_speed_mps": 1.3,
            "max_steering_angle_rad": 0.5235987756,
            "max_valid_for_s": 0.7,
            "max_future_skew_s": 0.1,
            "watchdog_period_s": 0.05,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._monotonic_clock = monotonic_clock
        self._consumer = CanonicalCommandConsumer(
            CanonicalCommandConfig(
                **{
                    name: self.get_parameter(name).value
                    for name in (
                        "max_forward_speed_mps",
                        "max_reverse_speed_mps",
                        "max_steering_angle_rad",
                        "max_valid_for_s",
                        "max_future_skew_s",
                    )
                }
            )
        )
        period_s = float(self.get_parameter("watchdog_period_s").value)
        if period_s <= 0.0:
            raise ValueError("watchdog_period_s must be positive")
        self._accepted = 0
        self._rejected = 0
        self._timeouts = 0
        self._last_reason = "no_command"
        self._publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.create_subscription(
            VehicleCommand,
            str(self.get_parameter("input_topic").value),
            self._on_command,
            10,
        )
        self.create_timer(period_s, self._tick)

    def _on_command(self, message: VehicleCommand) -> None:
        sample = CanonicalCommandSample(
            stamp_ns=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec,
            source=int(message.source),
            drive_enabled=bool(message.drive_enabled),
            emergency_stop=bool(message.emergency_stop),
            brake_ratio=float(message.brake_ratio),
            speed_mps=float(message.drive.speed),
            steering_angle_rad=float(message.drive.steering_angle),
            steering_angle_velocity_rad_s=float(message.drive.steering_angle_velocity),
            acceleration_mps2=float(message.drive.acceleration),
            jerk_mps3=float(message.drive.jerk),
            valid_for_s=message.valid_for.sec + message.valid_for.nanosec / 1_000_000_000.0,
        )
        effective = self._consumer.ingest(
            sample,
            ros_now_ns=self.get_clock().now().nanoseconds,
            monotonic_now_s=self._monotonic_clock(),
        )
        if effective.valid:
            self._accepted += 1
        else:
            self._rejected += 1
        self._last_reason = effective.reason
        self._publish_diagnostics(effective)

    def _tick(self) -> None:
        before = self._consumer.effective.reason
        effective = self._consumer.tick(self._monotonic_clock())
        if effective.reason == "watchdog_timeout" and before != effective.reason:
            self._timeouts += 1
        self._last_reason = effective.reason
        self._publish_diagnostics(effective)

    def _publish_diagnostics(self, effective) -> None:
        status = DiagnosticStatus()
        status.name = "salus_control/canonical_command_dry_run"
        status.hardware_id = "dry_run"
        status.level = DiagnosticStatus.OK if effective.valid else DiagnosticStatus.ERROR
        status.message = effective.reason
        values = {
            "accepted": self._accepted,
            "rejected": self._rejected,
            "watchdog_timeouts": self._timeouts,
            "source": effective.source,
            "drive_enabled": effective.drive_enabled,
            "emergency_stop": effective.emergency_stop,
            "brake_ratio": effective.brake_ratio,
            "speed_mps": effective.speed_mps,
            "steering_angle_rad": effective.steering_angle_rad,
            "reason": self._last_reason,
            "backend": "dry_run",
            "authoritative": "false",
        }
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        diagnostics.status = [status]
        self._publisher.publish(diagnostics)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CanonicalCommandDryRunNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
