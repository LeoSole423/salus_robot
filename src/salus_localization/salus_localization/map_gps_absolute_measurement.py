"""Publish absolute GPS position in the fixed simulation map frame."""
from __future__ import annotations

import math
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix


def project_fix(latitude: float, longitude: float, datum_lat: float, datum_lon: float) -> tuple[float, float]:
    north = (latitude - datum_lat) * 111_320.0
    east = (longitude - datum_lon) * 111_320.0 * math.cos(math.radians(datum_lat))
    return east, north


class MapGpsAbsoluteMeasurement(Node):
    def __init__(self) -> None:
        super().__init__("map_gps_absolute_measurement")
        self.declare_parameter("gps_topic", "/gps/fix")
        self.declare_parameter("output_topic", "/gps/odometry_map")
        self.declare_parameter("datum_lat", -31.4858037)
        self.declare_parameter("datum_lon", -64.2410570)
        self.declare_parameter("covariance_xy", 0.05)
        self.publisher = self.create_publisher(Odometry, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(NavSatFix, str(self.get_parameter("gps_topic").value), self.on_fix, 10)
        # This simulation owns a fixed datum.  Exposing its conversion through
        # the standard robot_localization service keeps navigation independent
        # from a transient navsat_transform lifecycle service.
        self.create_service(FromLL, "/fromLL", self.on_from_ll)

    def on_fix(self, fix: NavSatFix) -> None:
        if not math.isfinite(fix.latitude) or not math.isfinite(fix.longitude): return
        message = Odometry(); message.header = fix.header; message.header.frame_id = "map"
        message.pose.pose.position.x, message.pose.pose.position.y = project_fix(fix.latitude, fix.longitude, float(self.get_parameter("datum_lat").value), float(self.get_parameter("datum_lon").value))
        message.pose.pose.orientation.w = 1.0; covariance = [0.0] * 36; covariance[0] = covariance[7] = float(self.get_parameter("covariance_xy").value); covariance[14] = covariance[21] = covariance[28] = covariance[35] = 1e6; message.pose.covariance = covariance
        self.publisher.publish(message)

    def on_from_ll(self, request: FromLL.Request, response: FromLL.Response) -> FromLL.Response:
        response.map_point.x, response.map_point.y = project_fix(
            request.ll_point.latitude,
            request.ll_point.longitude,
            float(self.get_parameter("datum_lat").value),
            float(self.get_parameter("datum_lon").value),
        )
        response.map_point.z = 0.0
        return response


def main(args=None) -> None:
    rclpy.init(args=args); node = MapGpsAbsoluteMeasurement()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
