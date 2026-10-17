import json
import time

import pytest
import rclpy
from rclpy.parameter import Parameter

from salus_hardware.bmi088_domain import Calibration, decode
from salus_hardware.bmi088_node import Bmi088Node
from salus_hardware.imu_comparison_node import ImuComparisonNode
from sensor_msgs.msg import Imu
from test_bmi088_domain import packet, profile


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_node_maps_time_calibration_unknown_orientation_and_stale_status():
    rclpy.init()
    node = Bmi088Node(
        parameter_overrides=[Parameter('port', value='/test/only')], start_worker=False)
    try:
        node.calibration = Calibration.parse(profile(), 'board-1')
        pubs = {key: CapturingPublisher() for key in node.publishers_by_key}
        node.publishers_by_key = pubs
        node.temperature_pub = CapturingPublisher()
        node.status_pub = CapturingPublisher()
        received = time.monotonic_ns()
        ros_now = node.get_clock().now().nanoseconds
        sample = decode(packet())
        node.process(sample, received-3000000, received, ros_now)
        raw, corrected = pubs['data_raw'].messages[0], pubs['data'].messages[0]
        assert raw.header.frame_id == 'bmi088_link'
        assert raw.orientation_covariance[0] == -1
        assert raw.linear_acceleration_covariance[0] == -1
        assert abs(corrected.angular_velocity.z) < 1e-8
        assert raw.angular_velocity.z > 0.02
        assert corrected.angular_velocity_covariance[8] >= 0.0001
        accel = pubs['accel'].messages[0]
        assert accel.angular_velocity_covariance[0] == -1
        gyro_ns = corrected.header.stamp.sec*1000000000+corrected.header.stamp.nanosec
        accel_ns = accel.header.stamp.sec*1000000000+accel.header.stamp.nanosec
        assert gyro_ns-accel_ns == 1000000
        with pytest.raises(ValueError):
            node.process(sample, received-3000000, received, ros_now)
        node.last_good = node.last_gyro_good = node.last_accel_good = time.monotonic()-1
        node._status()
        status = json.loads(node.status_pub.messages[-1].data)
        assert not status['fresh'] and not status['gyro_fresh']
        assert not status['operational_source_changed']
        assert not status['hardware_synchronized']
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_comparison_requires_valid_common_frame_and_rejects_stale():
    rclpy.init()
    node = ImuComparisonNode()
    node.publisher = CapturingPublisher()
    try:
        candidate, reference = Imu(), Imu()
        now = node.get_clock().now().to_msg()
        for msg in (candidate, reference):
            msg.header.stamp = now
            msg.header.frame_id = 'base_footprint'
        candidate.angular_velocity.z = 0.5
        reference.angular_velocity.z = 0.4
        node._accept('candidate', candidate)
        node._accept('reference', reference)
        node._pair()
        node._report()
        report = json.loads(node.publisher.messages[-1].data)
        assert report['fresh'] and report['pairs'] == 1
        assert report['mean_error_rad_s'][2] == pytest.approx(0.1)
        assert not report['hardware_parity']
        old = Imu()
        old.header.stamp.sec = 1
        old.header.frame_id = 'base_footprint'
        node._accept('candidate', old)
        assert node.counts['rejected'] == 1
        # New pair in an unknown sensor frame must wait for TF.
        candidate.header.stamp = node.get_clock().now().to_msg()
        candidate.header.frame_id = 'missing_mount'
        reference.header.stamp = candidate.header.stamp
        node._accept('reference', reference)
        node._accept('candidate', candidate)
        node._pair()
        assert 'tf' in node.reason
    finally:
        node.destroy_node()
        rclpy.shutdown()
