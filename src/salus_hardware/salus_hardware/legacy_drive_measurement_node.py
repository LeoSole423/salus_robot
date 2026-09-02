"""Read-only ROS adapter from legacy ``DriveTelemetry`` to measurements."""

from __future__ import annotations

import math

from interfaces.msg import DriveTelemetry
import rclpy
from rclpy.node import Node

from salus_interfaces.msg import (
    SteeringMeasurement,
    TractionMeasurement,
)

from .legacy_drive_measurement_domain import (
    LegacyDriveSample,
    SteeringMeasurementValue,
    TractionMeasurementValue,
    adapt_legacy_drive_sample,
)


class LegacyDriveMeasurementNode(Node):
    """Publishes canonical observations without commanding any hardware."""

    def __init__(self) -> None:
        super().__init__("legacy_drive_measurement_adapter")
        self.declare_parameter("legacy_telemetry_topic", "/controller/drive_telemetry")
        self.declare_parameter("traction_topic", "/vehicle/measurements/traction")
        self.declare_parameter("steering_topic", "/vehicle/measurements/steering")
        self.declare_parameter("traction_source_id", "rear_traction_motor")
        self.declare_parameter("steering_source_id", "front_steering_linkage")
        self._traction_source_id = _required_string_parameter(self, "traction_source_id")
        self._steering_source_id = _required_string_parameter(self, "steering_source_id")
        self._sequence = 0
        self._traction_publisher = self.create_publisher(
            TractionMeasurement,
            str(self.get_parameter("traction_topic").value),
            10,
        )
        self._steering_publisher = self.create_publisher(
            SteeringMeasurement,
            str(self.get_parameter("steering_topic").value),
            10,
        )
        self._subscription = self.create_subscription(
            DriveTelemetry,
            str(self.get_parameter("legacy_telemetry_topic").value),
            self._on_telemetry,
            10,
        )

    def _on_telemetry(self, message: DriveTelemetry) -> None:
        traction, steering = adapt_legacy_drive_sample(_sample_from_ros(message))
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        self._traction_publisher.publish(_traction_message(
            message,
            self._traction_source_id,
            self._sequence,
            traction,
        ))
        self._steering_publisher.publish(_steering_message(
            message,
            self._steering_source_id,
            self._sequence,
            steering,
        ))


def _sample_from_ros(message: DriveTelemetry) -> LegacyDriveSample:
    return LegacyDriveSample(
        fresh=bool(message.fresh),
        reverse_requested=bool(message.reverse_requested),
        speed_valid=bool(message.speed_valid),
        steer_valid=bool(message.steer_valid),
        speed_mps_measured=float(message.speed_mps_measured),
        steer_deg_measured=float(message.steer_deg_measured),
    )


def _traction_message(
    legacy: DriveTelemetry,
    source_id: str,
    sequence: int,
    value: TractionMeasurementValue,
) -> TractionMeasurement:
    message = TractionMeasurement()
    _fill_metadata(message.metadata, legacy, source_id, sequence, value.fields)
    message.source_type = value.source_type
    _fill_fields(message, value.fields)
    message.position_rad = math.nan
    message.angular_velocity_rad_s = math.nan
    message.linear_velocity_mps = value.linear_velocity_mps
    return message


def _steering_message(
    legacy: DriveTelemetry,
    source_id: str,
    sequence: int,
    value: SteeringMeasurementValue,
) -> SteeringMeasurement:
    message = SteeringMeasurement()
    _fill_metadata(message.metadata, legacy, source_id, sequence, value.fields)
    message.source_type = value.source_type
    _fill_fields(message, value.fields)
    message.position_rad = value.position_rad
    message.angular_velocity_rad_s = math.nan
    return message


def _fill_metadata(
    metadata: object,
    legacy: DriveTelemetry,
    source_id: str,
    sequence: int,
    fields: object,
) -> None:
    metadata.header.stamp = legacy.stamp
    metadata.source_id = source_id
    metadata.status = fields.status
    metadata.sequence = sequence


def _fill_fields(message: object, fields: object) -> None:
    message.available_fields = fields.available_fields
    message.measured_fields = fields.measured_fields
    message.calculated_fields = fields.calculated_fields
    message.inferred_fields = fields.inferred_fields


def _required_string_parameter(node: Node, name: str) -> str:
    value = str(node.get_parameter(name).value).strip()
    if not value:
        raise ValueError(f"{name} must not be empty")
    return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LegacyDriveMeasurementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
