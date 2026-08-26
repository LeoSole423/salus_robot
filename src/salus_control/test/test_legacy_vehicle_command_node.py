import math

import pytest
import rclpy
from rclpy.parameter import Parameter
from salus_interfaces.msg import CmdVelFinal

from salus_control.control_logic import COMMAND_SOURCE_AUTO, COMMAND_SOURCE_SAFETY
from salus_control.legacy_vehicle_command_node import LegacyVehicleCommandNode


class PublisherCapture:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


@pytest.fixture
def node():
    rclpy.init()
    instance = LegacyVehicleCommandNode(
        parameter_overrides=[Parameter("valid_for_s", value=0.7)]
    )
    capture = PublisherCapture()
    instance._publisher = capture
    try:
        yield instance, capture
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def test_node_stamps_and_publishes_the_canonical_message(node) -> None:
    instance, capture = node
    incoming = CmdVelFinal()
    incoming.twist.linear.x = 2.0
    incoming.twist.angular.z = 0.4
    incoming.source = COMMAND_SOURCE_AUTO

    instance._on_command(incoming)

    assert len(capture.messages) == 1
    outgoing = capture.messages[0]
    assert outgoing.header.frame_id == "base_footprint"
    assert outgoing.header.stamp.sec > 0
    assert outgoing.valid_for.sec == 0
    assert outgoing.valid_for.nanosec == 700_000_000
    assert outgoing.source == COMMAND_SOURCE_AUTO
    assert outgoing.drive_enabled
    assert not outgoing.emergency_stop
    assert outgoing.brake_ratio == 0.0
    assert outgoing.drive.speed == pytest.approx(2.0)
    assert outgoing.drive.steering_angle == pytest.approx(math.atan(0.94 * 0.2))


def test_node_publishes_safe_message_for_invalid_input(node) -> None:
    instance, capture = node
    incoming = CmdVelFinal()
    incoming.twist.linear.x = math.nan
    incoming.source = COMMAND_SOURCE_AUTO

    instance._on_command(incoming)

    outgoing = capture.messages[0]
    assert outgoing.source == COMMAND_SOURCE_SAFETY
    assert not outgoing.drive_enabled
    assert outgoing.emergency_stop
    assert outgoing.brake_ratio == 1.0
    assert outgoing.drive.speed == 0.0
