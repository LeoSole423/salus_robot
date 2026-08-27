from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry

from salus_localization.sim_external_heading_from_odom import (
    SimExternalHeadingFromOdomNode,
)


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_fixture_publishes_only_planar_orientation_with_source_timestamp() -> None:
    rclpy.init()
    node = SimExternalHeadingFromOdomNode()
    try:
        publisher = CapturingPublisher()
        node._publisher = publisher
        odometry = Odometry()
        odometry.header.stamp.sec = 4
        odometry.pose.pose.orientation.z = 0.2
        odometry.pose.pose.orientation.w = 0.979795897
        node._on_odometry(odometry)
        assert len(publisher.messages) == 1
        message = publisher.messages[0]
        assert message.header.stamp.sec == 4
        assert message.header.frame_id == "base_footprint"
        assert message.orientation_covariance[8] == 0.02
        assert message.angular_velocity_covariance[0] == -1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_console_entry_point_is_packaged() -> None:
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "sim_external_heading_from_odom = " in setup
