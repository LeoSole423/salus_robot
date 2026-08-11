"""Normalize a simulated IMU independently from GPS and LiDAR processing."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


DEFAULT_IMU_ORIENTATION_VARIANCE = 0.01
DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE = 0.01
DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE = 0.1


def covariance_is_zero(values) -> bool:
    return all(abs(float(value)) <= 1.0e-12 for value in values)


def default_covariance(variance: float) -> list[float]:
    return [variance, 0.0, 0.0, 0.0, variance, 0.0, 0.0, 0.0, variance]


class ImuNormalizerNode(Node):
    """Set the canonical frame and safe covariances on simulated IMU messages."""

    def __init__(self) -> None:
        super().__init__("imu_normalizer")
        self.declare_parameter("input_topic", "/imu/data_raw")
        self.declare_parameter("output_topic", "/imu/data")
        self.declare_parameter("frame_id", "imu_link")
        self._frame_id = str(self.get_parameter("frame_id").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        self._publisher = self.create_publisher(Imu, output_topic, 10)
        self.create_subscription(Imu, input_topic, self._on_imu, 10)

    def _on_imu(self, msg: Imu) -> None:
        out = deepcopy(msg)
        out.header.frame_id = self._frame_id
        if covariance_is_zero(out.orientation_covariance):
            out.orientation_covariance = default_covariance(DEFAULT_IMU_ORIENTATION_VARIANCE)
        if covariance_is_zero(out.angular_velocity_covariance):
            out.angular_velocity_covariance = default_covariance(DEFAULT_IMU_ANGULAR_VELOCITY_VARIANCE)
        if covariance_is_zero(out.linear_acceleration_covariance):
            out.linear_acceleration_covariance = default_covariance(DEFAULT_IMU_LINEAR_ACCELERATION_VARIANCE)
        self._publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNormalizerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
