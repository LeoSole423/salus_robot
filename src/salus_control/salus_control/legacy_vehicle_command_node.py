"""ROS shadow publisher for the canonical vehicle command contract."""

from __future__ import annotations

import rclpy
from interfaces.msg import CmdVelFinal as LegacyCmdVelFinal
from rclpy.node import Node
from salus_interfaces.msg import CmdVelFinal as CanonicalCmdVelFinal, VehicleCommand

from .legacy_vehicle_command import (
    LegacyVehicleCommandConfig,
    translate_legacy_command,
)


_INPUT_WIRE_TYPES = {
    "salus_interfaces": CanonicalCmdVelFinal,
    "interfaces": LegacyCmdVelFinal,
}


class LegacyVehicleCommandNode(Node):
    """Observe the authoritative legacy command without controlling a backend."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "legacy_vehicle_command_adapter", parameter_overrides=parameter_overrides
        )
        defaults = {
            "input_topic": "/cmd_vel_final",
            "input_wire_type": "salus_interfaces",
            "output_topic": "/vehicle/command_shadow",
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
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._input_wire_type, self._input_message_type = _input_message_type(self)
        self._frame_id = str(self.get_parameter("frame_id").value).strip()
        if not self._frame_id:
            raise ValueError("frame_id must not be empty")
        self._config = LegacyVehicleCommandConfig(
            **{
                name: self.get_parameter(name).value
                for name in defaults
                if name not in {
                    "input_topic",
                    "input_wire_type",
                    "output_topic",
                    "frame_id",
                }
            }
        )
        self._publisher = self.create_publisher(
            VehicleCommand, str(self.get_parameter("output_topic").value), 10
        )
        self._subscription = self.create_subscription(
            self._input_message_type,
            str(self.get_parameter("input_topic").value),
            self._on_command,
            10,
        )

    def _on_command(self, incoming: object) -> None:
        value = translate_legacy_command(
            linear_x_mps=float(incoming.twist.linear.x),
            angular_z_rps=float(incoming.twist.angular.z),
            brake_pct=int(incoming.brake_pct),
            source=int(incoming.source),
            config=self._config,
        )
        outgoing = VehicleCommand()
        outgoing.header.stamp = self.get_clock().now().to_msg()
        outgoing.header.frame_id = self._frame_id
        duration_ns = round(value.valid_for_s * 1_000_000_000)
        seconds, nanoseconds = divmod(duration_ns, 1_000_000_000)
        outgoing.valid_for.sec = seconds
        outgoing.valid_for.nanosec = nanoseconds
        outgoing.source = value.source
        outgoing.drive_enabled = value.drive_enabled
        outgoing.emergency_stop = value.emergency_stop
        outgoing.brake_ratio = value.brake_ratio
        outgoing.drive.speed = value.speed_mps
        outgoing.drive.steering_angle = value.steering_angle_rad
        outgoing.drive.steering_angle_velocity = 0.0
        outgoing.drive.acceleration = 0.0
        outgoing.drive.jerk = 0.0
        self._publisher.publish(outgoing)
        if not value.valid_input:
            self.get_logger().error(
                f"publishing safe shadow command: {value.reason}",
                throttle_duration_sec=1.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LegacyVehicleCommandNode()
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
