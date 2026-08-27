from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu

from salus_localization.imu_selector import ImuSelectorNode


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _message(stamp=1):
    message = Imu()
    message.header.stamp.sec = stamp
    message.header.frame_id = "imu_secondary_link"
    message.orientation.w = 1.0
    message.orientation_covariance[0] = 0.1
    return message


def test_node_subscribes_only_selected_source_and_publishes_a_copy() -> None:
    rclpy.init()
    node = ImuSelectorNode(parameter_overrides=[
        Parameter("selected_source", value="imu_secondary"),
        Parameter("secondary_topic", value="/test/secondary"),
        Parameter("output_topic", value="/test/output"),
    ])
    try:
        publisher = CapturingPublisher()
        node._publisher = publisher
        assert node._subscription.topic_name == "/test/secondary"
        message = _message()
        node._on_imu(message)
        assert len(publisher.messages) == 1
        assert publisher.messages[0] is not message
        node._on_imu(message)
        assert len(publisher.messages) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_console_entry_point_is_packaged() -> None:
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "imu_selector = salus_localization.imu_selector:main" in setup
