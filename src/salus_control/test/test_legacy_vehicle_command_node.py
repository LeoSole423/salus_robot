import math
import time

import pytest
import rclpy
from interfaces.msg import CmdVelFinal as LegacyCmdVelFinal
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from salus_interfaces.msg import CmdVelFinal as CanonicalCmdVelFinal, VehicleCommand

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
    assert instance._input_wire_type == "salus_interfaces"
    assert instance._input_message_type is CanonicalCmdVelFinal
    incoming = CanonicalCmdVelFinal()
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
    incoming = CanonicalCmdVelFinal()
    incoming.twist.linear.x = math.nan
    incoming.source = COMMAND_SOURCE_AUTO

    instance._on_command(incoming)

    outgoing = capture.messages[0]
    assert outgoing.source == COMMAND_SOURCE_SAFETY
    assert not outgoing.drive_enabled
    assert outgoing.emergency_stop
    assert outgoing.brake_ratio == 1.0
    assert outgoing.drive.speed == 0.0


@pytest.mark.parametrize(
    ("linear_x", "source", "expected_speed", "expected_source", "expected_brake"),
    [
        (2.0, COMMAND_SOURCE_AUTO, 2.0, COMMAND_SOURCE_AUTO, 0.0),
        (math.nan, COMMAND_SOURCE_AUTO, 0.0, COMMAND_SOURCE_SAFETY, 1.0),
    ],
)
def test_real_legacy_wire_publisher_reaches_safe_canonical_shadow(
    linear_x: float,
    source: int,
    expected_speed: float,
    expected_source: int,
    expected_brake: float,
) -> None:
    """Exercise the real DDS boundary with the legacy package identity."""
    rclpy.init()
    input_topic = "/test/legacy_cmd_vel_final"
    output_topic = "/test/vehicle_command_shadow"
    adapter = LegacyVehicleCommandNode(parameter_overrides=[
        Parameter("input_topic", value=input_topic),
        Parameter("input_wire_type", value="interfaces"),
        Parameter("output_topic", value=output_topic),
    ])
    publisher_node = Node("legacy_cmd_vel_final_wire_publisher")
    observer = Node("canonical_command_shadow_observer")
    executor = SingleThreadedExecutor()
    received = []
    try:
        assert adapter._input_wire_type == "interfaces"
        assert adapter._input_message_type is LegacyCmdVelFinal
        publisher = publisher_node.create_publisher(
            LegacyCmdVelFinal, input_topic, 10
        )
        observer.create_subscription(VehicleCommand, output_topic, received.append, 10)
        for ros_node in (adapter, publisher_node, observer):
            executor.add_node(ros_node)

        legacy = LegacyCmdVelFinal()
        legacy.twist.linear.x = linear_x
        legacy.twist.angular.z = 0.4
        legacy.brake_pct = 0
        legacy.source = source
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not received:
            publisher.publish(legacy)
            executor.spin_once(timeout_sec=0.05)

        assert received
        assert dict(publisher_node.get_topic_names_and_types())[input_topic] == [
            "interfaces/msg/CmdVelFinal"
        ]
        shadow = received[-1]
        assert shadow.header.stamp.sec > 0
        assert shadow.header.frame_id == "base_footprint"
        assert shadow.source == expected_source
        assert shadow.drive.speed == pytest.approx(expected_speed)
        assert shadow.brake_ratio == expected_brake
        if math.isnan(linear_x):
            assert not shadow.drive_enabled
            assert shadow.emergency_stop
        else:
            assert shadow.drive_enabled
            assert not shadow.emergency_stop
            assert shadow.drive.steering_angle == pytest.approx(math.atan(0.94 * 0.2))
    finally:
        for ros_node in (observer, publisher_node, adapter):
            executor.remove_node(ros_node)
            ros_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_unknown_input_wire_type_is_rejected() -> None:
    rclpy.init()
    try:
        with pytest.raises(ValueError, match="input_wire_type must be one of"):
            LegacyVehicleCommandNode(parameter_overrides=[
                Parameter("input_wire_type", value="unknown"),
            ])
    finally:
        rclpy.shutdown()
