import math

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.parameter import Parameter
from salus_interfaces.msg import CmdVelFinal, VehicleCommand

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
