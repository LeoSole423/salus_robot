import rclpy
from diagnostic_msgs.msg import DiagnosticStatus
from rclpy.parameter import Parameter
from salus_interfaces.msg import VehicleCommand

from salus_control.canonical_command_dry_run_node import CanonicalCommandDryRunNode


class PublisherCapture:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def values(message) -> dict[str, str]:
    return {item.key: item.value for item in message.status[0].values}


def test_node_accepts_then_times_out_without_actuation() -> None:
    monotonic = [5.0]
    rclpy.init()
    node = CanonicalCommandDryRunNode(
        parameter_overrides=[Parameter("max_valid_for_s", value=0.2)],
        monotonic_clock=lambda: monotonic[0],
    )
    capture = PublisherCapture()
    node._publisher = capture
    try:
        message = VehicleCommand()
        message.header.stamp = node.get_clock().now().to_msg()
        message.valid_for.sec = 1
        message.source = VehicleCommand.SOURCE_AUTO
        message.drive_enabled = True
        message.drive.speed = 2.0
        node._on_command(message)
        assert capture.messages[-1].status[0].level == DiagnosticStatus.OK
        assert values(capture.messages[-1])["backend"] == "dry_run"
        assert values(capture.messages[-1])["authoritative"] == "false"

        monotonic[0] += 0.21
        node._tick()
        assert capture.messages[-1].status[0].level == DiagnosticStatus.ERROR
        assert values(capture.messages[-1])["reason"] == "watchdog_timeout"
        assert values(capture.messages[-1])["watchdog_timeouts"] == "1"
    finally:
        node.destroy_node()
        rclpy.shutdown()
