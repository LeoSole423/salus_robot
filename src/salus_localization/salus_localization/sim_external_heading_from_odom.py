"""Simulation-only external heading fixture derived from Gazebo ground truth."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class SimExternalHeadingFromOdomNode(Node):
    """Publish an absolute yaw sample; this is not a physical dual-GNSS model."""

    def __init__(self) -> None:
        super().__init__("sim_external_heading_from_odom")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("output_topic", "/heading/external")
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("yaw_variance_rad2", 0.02)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._yaw_variance = float(self.get_parameter("yaw_variance_rad2").value)
        if self._yaw_variance <= 0.0:
            raise ValueError("yaw_variance_rad2 must be positive")
        self._publisher = self.create_publisher(
            Imu,
            str(self.get_parameter("output_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odometry,
            qos_profile_sensor_data,
        )

    def _on_odometry(self, message: Odometry) -> None:
        output = Imu()
        output.header = deepcopy(message.header)
        output.header.frame_id = self._frame_id
        output.orientation = deepcopy(message.pose.pose.orientation)
        # Roll and pitch are deliberately unavailable; only planar yaw is modeled.
        output.orientation_covariance[0] = 1.0e6
        output.orientation_covariance[4] = 1.0e6
        output.orientation_covariance[8] = self._yaw_variance
        output.angular_velocity_covariance[0] = -1.0
        output.linear_acceleration_covariance[0] = -1.0
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimExternalHeadingFromOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
