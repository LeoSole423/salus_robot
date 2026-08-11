"""Normalize Gazebo NavSat messages and apply a deterministic receiver profile."""
from __future__ import annotations

from copy import deepcopy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from .gps_profiles import SimGpsFixProcessor, resolve_gps_profile


class SimGpsNormalizer(Node):
    def __init__(self) -> None:
        super().__init__("sim_gps_normalizer")
        self.declare_parameter("input_topic", "/gps/fix_raw")
        self.declare_parameter("output_topic", "/gps/fix")
        self.declare_parameter("rtk_status_topic", "/gps/rtk_status")
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("gps_profile", "f9p_rtk")
        self.declare_parameter("random_seed", 123)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.processor = SimGpsFixProcessor(resolve_gps_profile(str(self.get_parameter("gps_profile").value)), int(self.get_parameter("random_seed").value))
        self.publisher = self.create_publisher(NavSatFix, str(self.get_parameter("output_topic").value), 10)
        self.status_publisher = self.create_publisher(String, str(self.get_parameter("rtk_status_topic").value), 10)
        self.create_subscription(NavSatFix, str(self.get_parameter("input_topic").value), self.on_fix, 10)

    def on_fix(self, message: NavSatFix) -> None:
        output = self.processor.process(message)
        if output is None:
            return
        output.header.frame_id = self.frame_id
        self.publisher.publish(output)
        self.status_publisher.publish(String(data=self.processor.profile.rtk_status))


def main(args=None) -> None:
    rclpy.init(args=args); node = SimGpsNormalizer()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
