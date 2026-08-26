"""ROS adapter for explicitly calibrated vehicle kinematic conversions."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from salus_interfaces.msg import SteeringMeasurement, TractionMeasurement

from .kinematic_conversion_domain import (
    ConversionConfig,
    MeasurementInput,
    convert_steering,
    convert_traction,
)


class KinematicConversionNode(Node):
    """Convert selected physical sources without selecting or fusing them."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "vehicle_kinematic_converter", parameter_overrides=parameter_overrides
        )
        defaults = {
            "traction_input_topic": "/vehicle/measurements/traction",
            "steering_input_topic": "/vehicle/measurements/steering",
            "traction_output_topic": "/vehicle/kinematic_inputs/traction",
            "steering_output_topic": "/vehicle/kinematic_inputs/steering",
            "traction_input_source_id": "rear_traction_motor",
            "steering_input_source_id": "front_steering_linkage",
            "traction_output_source_id": "rear_drive_wheel_equivalent",
            "steering_output_source_id": "virtual_center_wheel",
            "output_frame": "base_footprint",
            "calibration_validated": False,
            "traction_linear_scale": 1.0,
            "steering_coefficients": [0.0, 1.0],
            "steering_limit_rad": 0.5235987756,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._traction_input_id = _required_string(self, "traction_input_source_id")
        self._steering_input_id = _required_string(self, "steering_input_source_id")
        self._traction_output_id = _required_string(self, "traction_output_source_id")
        self._steering_output_id = _required_string(self, "steering_output_source_id")
        self._output_frame = _required_string(self, "output_frame")
        self._config = ConversionConfig.create(
            calibration_validated=bool(self.get_parameter("calibration_validated").value),
            traction_linear_scale=float(self.get_parameter("traction_linear_scale").value),
            steering_coefficients=self.get_parameter("steering_coefficients").value,
            steering_limit_rad=float(self.get_parameter("steering_limit_rad").value),
        )
        self._traction_publisher = self.create_publisher(
            TractionMeasurement,
            str(self.get_parameter("traction_output_topic").value), 10,
        )
        self._steering_publisher = self.create_publisher(
            SteeringMeasurement,
            str(self.get_parameter("steering_output_topic").value), 10,
        )
        self._traction_subscription = self.create_subscription(
            TractionMeasurement,
            str(self.get_parameter("traction_input_topic").value),
            self._on_traction, 10,
        )
        self._steering_subscription = self.create_subscription(
            SteeringMeasurement,
            str(self.get_parameter("steering_input_topic").value),
            self._on_steering, 10,
        )

    def _on_traction(self, incoming: TractionMeasurement) -> None:
        if incoming.metadata.source_id != self._traction_input_id:
            return
        result = convert_traction(_traction_input(incoming), self._config)
        outgoing = TractionMeasurement()
        _metadata(outgoing, incoming, self._traction_output_id, self._output_frame, result)
        _fields(outgoing, result)
        outgoing.source_type = result.source_type
        outgoing.position_rad = math.nan
        outgoing.angular_velocity_rad_s = math.nan
        outgoing.linear_velocity_mps = result.value
        self._traction_publisher.publish(outgoing)

    def _on_steering(self, incoming: SteeringMeasurement) -> None:
        if incoming.metadata.source_id != self._steering_input_id:
            return
        result = convert_steering(_steering_input(incoming), self._config)
        outgoing = SteeringMeasurement()
        _metadata(outgoing, incoming, self._steering_output_id, self._output_frame, result)
        _fields(outgoing, result)
        outgoing.source_type = result.source_type
        outgoing.position_rad = result.value
        outgoing.angular_velocity_rad_s = math.nan
        self._steering_publisher.publish(outgoing)


def _traction_input(message: TractionMeasurement) -> MeasurementInput:
    return _input(message, message.linear_velocity_mps)


def _steering_input(message: SteeringMeasurement) -> MeasurementInput:
    return _input(message, message.position_rad)


def _input(message, value: float) -> MeasurementInput:
    return MeasurementInput(
        int(message.source_type), int(message.metadata.status),
        int(message.available_fields), int(message.measured_fields),
        int(message.calculated_fields), int(message.inferred_fields), float(value),
    )


def _metadata(outgoing, incoming, source_id: str, frame: str, result) -> None:
    outgoing.metadata.header.stamp = incoming.metadata.header.stamp
    outgoing.metadata.header.frame_id = frame
    outgoing.metadata.source_id = source_id
    outgoing.metadata.status = result.status
    outgoing.metadata.sequence = incoming.metadata.sequence


def _fields(message, result) -> None:
    message.available_fields = result.available_fields
    message.measured_fields = result.measured_fields
    message.calculated_fields = result.calculated_fields
    message.inferred_fields = result.inferred_fields


def _required_string(node: Node, name: str) -> str:
    value = str(node.get_parameter(name).value).strip()
    if not value:
        raise ValueError(f"{name} must not be empty")
    return value


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KinematicConversionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
