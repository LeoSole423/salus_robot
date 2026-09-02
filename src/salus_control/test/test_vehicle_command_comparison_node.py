import math
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import DiagnosticArray
from interfaces.msg import CmdVelFinal
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from salus_interfaces.msg import VehicleCommand

from salus_control.vehicle_command_comparison_node import (
    VehicleCommandComparisonNode,
)


class PublisherCapture:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def canonical_message(speed: float = 2.0) -> VehicleCommand:
    message = VehicleCommand()
    message.header.stamp.sec = 1
    message.header.frame_id = "base_footprint"
    message.valid_for.nanosec = 700_000_000
    message.source = VehicleCommand.SOURCE_AUTO
    message.drive_enabled = True
    message.drive.speed = speed
    message.drive.steering_angle = math.atan(0.94 * 0.2)
    return message


def legacy_message() -> CmdVelFinal:
    message = CmdVelFinal()
    message.twist.linear.x = 2.0
    message.twist.angular.z = 0.4
    message.source = VehicleCommand.SOURCE_AUTO
    return message


@pytest.fixture
def node():
    clock = [10.0]
    rclpy.init()
    instance = VehicleCommandComparisonNode(
        parameter_overrides=[Parameter("diagnostic_period_s", value=1.0)],
        monotonic_clock=lambda: clock[0],
    )
    capture = PublisherCapture()
    instance._publisher = capture
    try:
        yield instance, capture, clock
    finally:
        instance.destroy_node()
        rclpy.shutdown()


def diagnostic_values(message) -> dict[str, str]:
    return {item.key: item.value for item in message.status[0].values}


def test_reverse_callback_order_still_pairs_and_reports_ok(node) -> None:
    instance, capture, _ = node
    instance._on_shadow(canonical_message())
    instance._on_legacy(legacy_message())
    instance._publish_diagnostics()

    status = capture.messages[-1].status[0]
    assert status.level == DiagnosticStatus.OK
    assert diagnostic_values(capture.messages[-1])["compared"] == "1"
    assert diagnostic_values(capture.messages[-1])["matched"] == "1"
    assert diagnostic_values(capture.messages[-1])["authoritative"] == "false"


def test_divergence_is_latched_in_diagnostics(node) -> None:
    instance, capture, _ = node
    instance._on_legacy(legacy_message())
    instance._on_shadow(canonical_message(speed=1.0))
    instance._publish_diagnostics()

    status = capture.messages[-1].status[0]
    assert status.level == DiagnosticStatus.ERROR
    assert diagnostic_values(capture.messages[-1])["diverged"] == "1"
    assert diagnostic_values(capture.messages[-1])["last_reasons"] == (
        "speed_mps_mismatch"
    )


def test_unpaired_sample_times_out_monotonically(node) -> None:
    instance, capture, clock = node
    instance._on_legacy(legacy_message())
    clock[0] += 0.6
    instance._tick()

    status = capture.messages[-1].status[0]
    assert status.level == DiagnosticStatus.ERROR
    assert diagnostic_values(capture.messages[-1])["legacy_without_shadow"] == "1"
    assert diagnostic_values(capture.messages[-1])["last_reasons"] == "shadow_missing"


def test_real_legacy_wire_and_shadow_publishers_report_match() -> None:
    """Pair actual legacy and canonical ROS messages through the DDS graph."""
    rclpy.init()
    legacy_topic = "/test/legacy_cmd_vel_final"
    shadow_topic = "/test/vehicle_command_shadow"
    diagnostics_topic = "/test/vehicle_command_shadow/diagnostics"
    comparator = VehicleCommandComparisonNode(parameter_overrides=[
        Parameter("legacy_topic", value=legacy_topic),
        Parameter("shadow_topic", value=shadow_topic),
        Parameter("diagnostics_topic", value=diagnostics_topic),
        Parameter("diagnostic_period_s", value=0.05),
    ])
    legacy_publisher_node = Node("legacy_comparison_wire_publisher")
    shadow_publisher_node = Node("canonical_comparison_shadow_publisher")
    observer = Node("comparison_diagnostics_observer")
    executor = SingleThreadedExecutor()
    diagnostics = []
    try:
        legacy_publisher = legacy_publisher_node.create_publisher(
            CmdVelFinal, legacy_topic, 10
        )
        shadow_publisher = shadow_publisher_node.create_publisher(
            VehicleCommand, shadow_topic, 10
        )
        observer.create_subscription(DiagnosticArray, diagnostics_topic, diagnostics.append, 10)
        for ros_node in (
            comparator,
            legacy_publisher_node,
            shadow_publisher_node,
            observer,
        ):
            executor.add_node(ros_node)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not diagnostics:
            legacy_publisher.publish(legacy_message())
            shadow_publisher.publish(canonical_message())
            executor.spin_once(timeout_sec=0.05)

        assert diagnostics
        assert dict(legacy_publisher_node.get_topic_names_and_types())[legacy_topic] == [
            "interfaces/msg/CmdVelFinal"
        ]
        status = diagnostics[-1].status[0]
        values = diagnostic_values(diagnostics[-1])
        assert status.level == DiagnosticStatus.OK
        assert values["compared"] != "0"
        assert values["matched"] == values["compared"]
        assert values["authoritative"] == "false"
    finally:
        for ros_node in (
            observer,
            shadow_publisher_node,
            legacy_publisher_node,
            comparator,
        ):
            executor.remove_node(ros_node)
            ros_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
