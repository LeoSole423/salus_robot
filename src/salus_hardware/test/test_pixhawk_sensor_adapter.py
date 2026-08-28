from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from salus_hardware.pixhawk_sensor_adapter import PixhawkSensorAdapterNode


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _imu() -> Imu:
    message = Imu()
    message.header.stamp.sec = 1
    message.header.frame_id = "base_link"
    message.orientation.w = 1.0
    return message


def _no_fix() -> NavSatFix:
    message = NavSatFix()
    message.header.stamp.sec = 1
    message.header.frame_id = "base_link"
    message.status.status = NavSatStatus.STATUS_NO_FIX
    return message


def test_adapter_routes_valid_samples_without_changing_messages() -> None:
    rclpy.init()
    node = PixhawkSensorAdapterNode(parameter_overrides=[
        Parameter("imu_input_topic", value="/test/mavros/imu"),
        Parameter("gnss_input_topic", value="/test/mavros/fix"),
    ])
    try:
        imu_publisher = CapturingPublisher()
        gnss_publisher = CapturingPublisher()
        node._imu_publisher = imu_publisher
        node._gnss_publisher = gnss_publisher
        imu = _imu()
        fix = _no_fix()
        node._on_imu(imu)
        node._on_gnss(fix)
        assert imu_publisher.messages[0] is not imu
        assert imu_publisher.messages[0] == imu
        assert gnss_publisher.messages[0] is not fix
        assert gnss_publisher.messages[0] == fix
        assert node._imu_subscription.topic_name == "/test/mavros/imu"
        assert node._gnss_subscription.topic_name == "/test/mavros/fix"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_console_entry_point_is_packaged() -> None:
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "pixhawk_sensor_adapter = salus_hardware.pixhawk_sensor_adapter:main" in setup
