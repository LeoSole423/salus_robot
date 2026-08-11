"""Normalize a simulator-scoped cloud frame into the canonical LiDAR frame."""
from __future__ import annotations
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
class CloudNormalizer(Node):
    def __init__(self)->None:
        super().__init__("cloud_normalizer");self.declare_parameter("input_topic","/scan_3d_raw");self.declare_parameter("output_topic","/scan_3d");self.declare_parameter("output_frame_id","lidar_link");self.frame_id=str(self.get_parameter("output_frame_id").value);self.pub=self.create_publisher(PointCloud2,str(self.get_parameter("output_topic").value),qos_profile_sensor_data);self.create_subscription(PointCloud2,str(self.get_parameter("input_topic").value),self.on_cloud,qos_profile_sensor_data)
    def on_cloud(self,msg:PointCloud2)->None:
        normalized=deepcopy(msg);normalized.header.frame_id=self.frame_id;self.pub.publish(normalized)
def main(args=None)->None:
    rclpy.init(args=args);node=CloudNormalizer()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
