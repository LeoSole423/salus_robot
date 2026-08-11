"""Generate deterministic simulated IMU samples from Gazebo odometry.

This is deliberately a simulation adapter: it is not used with physical IMU
hardware and it publishes no TF.
"""

from __future__ import annotations

from copy import deepcopy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def planar_yaw_from_quaternion(quaternion) -> float:
    """Extract the planar yaw angle from a ROS quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


class SimImuFromOdomNode(Node):
    """Convert `/odom_raw` orientation and yaw rate into `/imu/data_raw`."""

    def __init__(self) -> None:
        super().__init__("sim_imu_from_odom")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("frame_id", "imu_link")
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._publisher = self.create_publisher(Imu, str(self.get_parameter("imu_topic").value), 10)
        self.create_subscription(Odometry, str(self.get_parameter("odom_topic").value), self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        imu = Imu()
        imu.header = deepcopy(msg.header)
        imu.header.frame_id = self._frame_id
        imu.orientation = deepcopy(msg.pose.pose.orientation)
        imu.angular_velocity.z = float(msg.twist.twist.angular.z)
        # The current motion simulation is planar and does not model acceleration.
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        self._publisher.publish(imu)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimImuFromOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
