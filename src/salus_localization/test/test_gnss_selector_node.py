from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix, NavSatStatus

from salus_localization.gnss_selector import GnssSelectorNode


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _no_fix() -> NavSatFix:
    message = NavSatFix()
    message.header.stamp.sec = 1
    message.header.frame_id = "base_link"
    message.status.status = NavSatStatus.STATUS_NO_FIX
    return message


def test_node_subscribes_only_selected_source_and_publishes_a_copy() -> None:
    rclpy.init()
    node = GnssSelectorNode(parameter_overrides=[
        Parameter("primary_topic", value="/test/primary"),
        Parameter("output_topic", value="/test/output"),
    ])
    try:
        publisher = CapturingPublisher()
        node._publisher = publisher
        assert node._subscription.topic_name == "/test/primary"
        message = _no_fix()
        node._on_fix(message)
        assert len(publisher.messages) == 1
        assert publisher.messages[0] is not message
        node._on_fix(message)
        assert len(publisher.messages) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_console_entry_point_is_packaged() -> None:
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "gnss_selector = salus_localization.gnss_selector:main" in setup
