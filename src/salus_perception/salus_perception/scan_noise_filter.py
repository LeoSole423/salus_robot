"""LaserScan cleanup adapter."""
from __future__ import annotations
from copy import deepcopy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from .scan_filters import clean_ranges

class ScanNoiseFilter(Node):
    def __init__(self) -> None:
        super().__init__("scan_noise_filter")
        for name, value in {"input_topic":"/scan", "output_topic":"/scan_clean", "range_min":0.4, "range_max":20.0, "speckle_window":2, "speckle_max_range":12.0, "max_deviation_m":0.30}.items(): self.declare_parameter(name, value)
        p=lambda n:self.get_parameter(n).value; self.kwargs={k:p(k) for k in ("range_min","range_max","speckle_window","speckle_max_range","max_deviation_m")}; self.pub=self.create_publisher(LaserScan,str(p("output_topic")),qos_profile_sensor_data); self.create_subscription(LaserScan,str(p("input_topic")),self.on_scan,qos_profile_sensor_data)
    def on_scan(self,msg: LaserScan)->None:
        result=deepcopy(msg); result.ranges=clean_ranges(msg.ranges,**self.kwargs); self.pub.publish(result)
def main(args=None)->None:
    rclpy.init(args=args); node=ScanNoiseFilter()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()
