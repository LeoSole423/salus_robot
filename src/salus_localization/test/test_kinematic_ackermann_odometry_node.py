from pathlib import Path

import rclpy
from salus_interfaces.msg import SteeringMeasurement, TractionMeasurement

from salus_localization.kinematic_ackermann_odometry import KinematicAckermannOdometryNode


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _traction(stamp=10):
    message = TractionMeasurement()
    message.metadata.source_id = "rear_drive_wheel_equivalent"
    message.metadata.header.stamp.sec = stamp
    message.metadata.status = message.metadata.STATUS_OK
    message.source_type = message.SOURCE_DRIVE_WHEEL
    message.available_fields = message.FIELD_LINEAR_VELOCITY
    message.calculated_fields = message.FIELD_LINEAR_VELOCITY
    message.linear_velocity_mps = 1.0
    return message


def _steering(stamp=10):
    message = SteeringMeasurement()
    message.metadata.source_id = "virtual_center_wheel"
    message.metadata.header.stamp.sec = stamp
    message.metadata.status = message.metadata.STATUS_OK
    message.source_type = message.SOURCE_VIRTUAL_CENTER_WHEEL
    message.available_fields = message.FIELD_POSITION
    message.calculated_fields = message.FIELD_POSITION
    message.position_rad = 0.1
    return message


def test_node_only_publishes_after_selected_valid_pair_and_never_tf():
    rclpy.init()
    node = KinematicAckermannOdometryNode()
    try:
        odom, twist = CapturingPublisher(), CapturingPublisher()
        node._odom_publisher, node._twist_publisher = odom, twist
        node._on_traction(_traction())
        assert odom.messages == []
        node._on_steering(_steering())
        assert len(odom.messages) == len(twist.messages) == 1
        result = odom.messages[0]
        assert result.header.frame_id == "odom" and result.child_frame_id == "base_footprint"
        assert result.pose.pose.position.x == 0.0 and result.twist.twist.linear.x == 1.0
        assert not hasattr(node, "_tf_broadcaster")
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_console_entry_point_is_packaged():
    setup = (Path(__file__).parents[1] / "setup.py").read_text(encoding="utf-8")
    assert "kinematic_ackermann_odometry = " in setup
