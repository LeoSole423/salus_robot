"""Non-authoritative diagnostics for the legacy command shadow."""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from interfaces.msg import CmdVelFinal as LegacyCmdVelFinal
from rclpy.node import Node
from salus_interfaces.msg import CmdVelFinal as CanonicalCmdVelFinal, VehicleCommand

from .legacy_vehicle_command import LegacyVehicleCommandConfig, translate_legacy_command
from .vehicle_command_comparison import (
    ComparisonTolerances,
    ObservedVehicleCommand,
    compare_vehicle_commands,
)


_INPUT_WIRE_TYPES = {
    "salus_interfaces": CanonicalCmdVelFinal,
    "interfaces": LegacyCmdVelFinal,
}


@dataclass(slots=True)
class PendingSample:
    value: object
    received_at_s: float


class VehicleCommandComparisonNode(Node):
    """Pair legacy and shadow samples and publish diagnostics only."""

    def __init__(self, *, parameter_overrides=None, monotonic_clock=time.monotonic):
        super().__init__(
            "vehicle_command_shadow_comparison", parameter_overrides=parameter_overrides
        )
        defaults = {
            "legacy_topic": "/cmd_vel_final",
            "input_wire_type": "salus_interfaces",
            "shadow_topic": "/vehicle/command_shadow",
            "diagnostics_topic": "/vehicle/command_shadow/diagnostics",
            "frame_id": "base_footprint",
            "max_speed_mps": 4.0,
            "max_reverse_mps": 1.3,
            "vx_deadband_mps": 0.1,
            "vx_min_effective_mps": 0.75,
            "wheelbase_m": 0.94,
            "steering_limit_rad": 0.5235987756,
            "operational_steering_limit_rad": 0.3141592654,
            "manual_operational_steering_limit_rad": 0.5235987756,
            "drive_enabled": True,
            "valid_for_s": 0.7,
            "speed_tolerance_mps": 1.0e-5,
            "steering_tolerance_rad": 1.0e-5,
            "brake_tolerance_ratio": 1.0e-5,
            "valid_for_tolerance_s": 1.0e-6,
            "pair_timeout_s": 0.5,
            "diagnostic_period_s": 0.5,
            "queue_depth": 50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._input_wire_type, self._input_message_type = _input_message_type(self)
        self._monotonic_clock = monotonic_clock
        self._config = LegacyVehicleCommandConfig(
            **{
                name: self.get_parameter(name).value
                for name in (
                    "max_speed_mps",
                    "max_reverse_mps",
                    "vx_deadband_mps",
                    "vx_min_effective_mps",
                    "wheelbase_m",
                    "steering_limit_rad",
                    "operational_steering_limit_rad",
                    "manual_operational_steering_limit_rad",
                    "drive_enabled",
                    "valid_for_s",
                )
            }
        )
        self._tolerances = ComparisonTolerances(
            speed_mps=float(self.get_parameter("speed_tolerance_mps").value),
            steering_angle_rad=float(
                self.get_parameter("steering_tolerance_rad").value
            ),
            brake_ratio=float(self.get_parameter("brake_tolerance_ratio").value),
            valid_for_s=float(self.get_parameter("valid_for_tolerance_s").value),
            expected_frame_id=str(self.get_parameter("frame_id").value),
        )
        self._pair_timeout_s = float(self.get_parameter("pair_timeout_s").value)
        queue_depth = int(self.get_parameter("queue_depth").value)
        diagnostic_period = float(self.get_parameter("diagnostic_period_s").value)
        if self._pair_timeout_s <= 0.0 or diagnostic_period <= 0.0 or queue_depth <= 0:
            raise ValueError("timeouts, period and queue_depth must be positive")
        self._legacy = deque(maxlen=queue_depth)
        self._shadow = deque(maxlen=queue_depth)
        self._compared = 0
        self._matched = 0
        self._diverged = 0
        self._legacy_timeouts = 0
        self._shadow_timeouts = 0
        self._legacy_dropped = 0
        self._shadow_dropped = 0
        self._last_reasons: tuple[str, ...] = ()
        self._publisher = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            10,
        )
        self.create_subscription(
            self._input_message_type,
            str(self.get_parameter("legacy_topic").value),
            self._on_legacy,
            10,
        )
        self.create_subscription(
            VehicleCommand,
            str(self.get_parameter("shadow_topic").value),
            self._on_shadow,
            10,
        )
        self.create_timer(diagnostic_period, self._tick)

    def _on_legacy(self, message: object) -> None:
        value = translate_legacy_command(
            linear_x_mps=float(message.twist.linear.x),
            angular_z_rps=float(message.twist.angular.z),
            brake_pct=int(message.brake_pct),
            source=int(message.source),
            config=self._config,
        )
        if len(self._legacy) == self._legacy.maxlen:
            self._legacy.popleft()
            self._legacy_dropped += 1
            self._last_reasons = ("legacy_queue_overflow",)
        self._legacy.append(PendingSample(value, self._monotonic_clock()))
        self._pair_available()

    def _on_shadow(self, message: VehicleCommand) -> None:
        value = ObservedVehicleCommand(
            source=int(message.source),
            drive_enabled=bool(message.drive_enabled),
            emergency_stop=bool(message.emergency_stop),
            brake_ratio=float(message.brake_ratio),
            speed_mps=float(message.drive.speed),
            steering_angle_rad=float(message.drive.steering_angle),
            valid_for_s=(
                float(message.valid_for.sec)
                + float(message.valid_for.nanosec) / 1_000_000_000.0
            ),
            frame_id=message.header.frame_id,
            stamp_ns=(
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            ),
        )
        if len(self._shadow) == self._shadow.maxlen:
            self._shadow.popleft()
            self._shadow_dropped += 1
            self._last_reasons = ("shadow_queue_overflow",)
        self._shadow.append(PendingSample(value, self._monotonic_clock()))
        self._pair_available()

    def _pair_available(self) -> None:
        while self._legacy and self._shadow:
            expected = self._legacy.popleft().value
            observed = self._shadow.popleft().value
            result = compare_vehicle_commands(expected, observed, self._tolerances)
            self._compared += 1
            if result.matches:
                self._matched += 1
            else:
                self._diverged += 1
                self._last_reasons = result.reasons

    def _tick(self) -> None:
        now_s = self._monotonic_clock()
        while self._legacy and now_s - self._legacy[0].received_at_s > self._pair_timeout_s:
            self._legacy.popleft()
            self._legacy_timeouts += 1
            self._last_reasons = ("shadow_missing",)
        while self._shadow and now_s - self._shadow[0].received_at_s > self._pair_timeout_s:
            self._shadow.popleft()
            self._shadow_timeouts += 1
            self._last_reasons = ("legacy_missing",)
        self._publish_diagnostics()

    def _publish_diagnostics(self) -> None:
        failures = (
            self._diverged
            + self._legacy_timeouts
            + self._shadow_timeouts
            + self._legacy_dropped
            + self._shadow_dropped
        )
        status = DiagnosticStatus()
        status.name = "salus_control/vehicle_command_shadow_comparison"
        status.hardware_id = "none"
        if failures:
            status.level = DiagnosticStatus.ERROR
            status.message = "shadow divergence detected"
        elif self._compared == 0:
            status.level = DiagnosticStatus.WARN
            status.message = "waiting for a comparable command pair"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "shadow matches legacy translation"
        values = {
            "compared": self._compared,
            "matched": self._matched,
            "diverged": self._diverged,
            "legacy_without_shadow": self._legacy_timeouts,
            "shadow_without_legacy": self._shadow_timeouts,
            "legacy_queue_dropped": self._legacy_dropped,
            "shadow_queue_dropped": self._shadow_dropped,
            "legacy_pending": len(self._legacy),
            "shadow_pending": len(self._shadow),
            "last_reasons": ",".join(self._last_reasons) or "none",
            "authoritative": "false",
        }
        status.values = [KeyValue(key=key, value=str(value)) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.status = [status]
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleCommandComparisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _input_message_type(node: Node) -> tuple[str, type]:
    wire_type = str(node.get_parameter("input_wire_type").value).strip()
    message_type = _INPUT_WIRE_TYPES.get(wire_type)
    if message_type is None:
        supported = ", ".join(sorted(_INPUT_WIRE_TYPES))
        raise ValueError(
            f"input_wire_type must be one of: {supported}; got {wire_type!r}"
        )
    return wire_type, message_type
