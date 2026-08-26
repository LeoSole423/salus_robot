import math
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter
from salus_hardware.kinematic_conversion_node import KinematicConversionNode
from salus_interfaces.msg import SteeringMeasurement, TractionMeasurement


class CapturingPublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


@pytest.fixture
def node():
    rclpy.init()
    instance = KinematicConversionNode()
    try:
        yield instance
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_default_unvalidated_node_publishes_unavailable_and_copies_lineage(node) -> None:
    publisher = CapturingPublisher()
    node._traction_publisher = publisher
    incoming = TractionMeasurement()
    incoming.metadata.source_id = "rear_traction_motor"
    incoming.metadata.header.stamp.sec = 12
    incoming.metadata.sequence = 41
    incoming.metadata.status = incoming.metadata.STATUS_OK
    incoming.source_type = incoming.SOURCE_MOTOR_SHAFT
    incoming.available_fields = incoming.FIELD_LINEAR_VELOCITY
    incoming.inferred_fields = incoming.FIELD_LINEAR_VELOCITY
    incoming.linear_velocity_mps = -1.0

    node._on_traction(incoming)

    result = publisher.messages[0]
    assert result.metadata.status == result.metadata.STATUS_UNAVAILABLE
    assert result.metadata.header.stamp.sec == 12
    assert result.metadata.header.frame_id == "base_footprint"
    assert result.metadata.sequence == 41
    assert result.available_fields == 0
    assert math.isnan(result.linear_velocity_mps)


def test_unselected_source_is_ignored(node) -> None:
    publisher = CapturingPublisher()
    node._steering_publisher = publisher
    incoming = SteeringMeasurement()
    incoming.metadata.source_id = "another_linkage"
    node._on_steering(incoming)
    assert publisher.messages == []


def test_validated_node_converts_both_selected_sources() -> None:
    rclpy.init()
    # Construct through ROS with overrides so parameter validation is exercised.
    node = KinematicConversionNode(
        parameter_overrides=[
            Parameter("calibration_validated", value=True),
            Parameter("traction_linear_scale", value=0.5),
            Parameter("steering_coefficients", value=[0.0, 2.0]),
            Parameter("steering_limit_rad", value=0.5),
        ]
    )
    try:
        traction_pub, steering_pub = CapturingPublisher(), CapturingPublisher()
        node._traction_publisher, node._steering_publisher = traction_pub, steering_pub
        traction = TractionMeasurement()
        traction.metadata.source_id = "rear_traction_motor"
        traction.metadata.status = traction.metadata.STATUS_OK
        traction.source_type = traction.SOURCE_MOTOR_SHAFT
        traction.available_fields = traction.FIELD_LINEAR_VELOCITY
        traction.inferred_fields = traction.FIELD_LINEAR_VELOCITY
        traction.linear_velocity_mps = 2.0
        steering = SteeringMeasurement()
        steering.metadata.source_id = "front_steering_linkage"
        steering.metadata.status = steering.metadata.STATUS_OK
        steering.source_type = steering.SOURCE_STEERING_LINKAGE
        steering.available_fields = steering.FIELD_POSITION
        steering.calculated_fields = steering.FIELD_POSITION
        steering.position_rad = 0.4
        node._on_traction(traction)
        node._on_steering(steering)
        assert traction_pub.messages[0].linear_velocity_mps == 1.0
        assert traction_pub.messages[0].inferred_fields == traction.FIELD_LINEAR_VELOCITY
        assert steering_pub.messages[0].position_rad == 0.5
        assert steering_pub.messages[0].calculated_fields == steering.FIELD_POSITION
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_console_entry_point_is_packaged() -> None:
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "vehicle_kinematic_converter = " in setup
