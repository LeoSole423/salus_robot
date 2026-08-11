"""Conservative heading estimate from consecutive GPS fixes."""
from __future__ import annotations

import json
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String


def heading_from_fixes(previous: NavSatFix, current: NavSatFix) -> tuple[float, float]:
    north = (current.latitude - previous.latitude) * 111_320.0
    east = (current.longitude - previous.longitude) * 111_320.0 * math.cos(math.radians(current.latitude))
    return math.atan2(north, east), math.hypot(north, east)


class GpsCourseHeading(Node):
    def __init__(self) -> None:
        super().__init__("gps_course_heading")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("output_topic", "/gps/course_heading")
        self.declare_parameter("debug_topic", "/gps/course_heading/debug")
        self.declare_parameter("min_distance_m", 2.0)
        self.declare_parameter("min_speed_mps", 0.8)
        self.previous = None
        self.output = self.create_publisher(Imu, str(self.get_parameter("output_topic").value), 10)
        self.debug = self.create_publisher(String, str(self.get_parameter("debug_topic").value), 10)
        self.create_subscription(NavSatFix, str(self.get_parameter("gps_topic").value), self.on_fix, 10)

    def on_fix(self, current: NavSatFix) -> None:
        if self.previous is None:
            self.previous = current; return
        yaw, distance = heading_from_fixes(self.previous, current)
        elapsed = (current.header.stamp.sec - self.previous.header.stamp.sec) + (current.header.stamp.nanosec - self.previous.header.stamp.nanosec) / 1e9
        self.previous = current
        speed = distance / elapsed if elapsed > 0 else 0.0
        valid = distance >= float(self.get_parameter("min_distance_m").value) and speed >= float(self.get_parameter("min_speed_mps").value)
        self.debug.publish(String(data=json.dumps({"valid": valid, "reason": "ok" if valid else "distance_or_speed", "distance_m": distance, "speed_mps": speed})))
        if not valid: return
        output = Imu(); output.header = current.header; output.orientation.w = math.cos(yaw / 2); output.orientation.z = math.sin(yaw / 2); output.orientation_covariance[8] = 0.05
        self.output.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args); node = GpsCourseHeading()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
