"""Stationary gates that prevent the global EKF from integrating false motion."""
from __future__ import annotations

from copy import deepcopy
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from salus_interfaces.msg import DriveTelemetry
from sensor_msgs.msg import Imu


def is_stationary(speed_mps: float, fresh: bool, speed_valid: bool, threshold_mps: float = 0.03) -> bool:
    return bool(fresh and speed_valid and math.isfinite(speed_mps) and abs(speed_mps) <= threshold_mps)


def zero_twist(msg: Odometry) -> Odometry:
    result = deepcopy(msg); result.twist.twist.linear.x = result.twist.twist.linear.y = result.twist.twist.linear.z = 0.0; result.twist.twist.angular.x = result.twist.twist.angular.y = result.twist.twist.angular.z = 0.0; return result


def zero_angular_velocity(msg: Imu) -> Imu:
    result = deepcopy(msg); result.angular_velocity.x = result.angular_velocity.y = result.angular_velocity.z = 0.0; return result


def yaw_only(msg: Odometry, variance: float = 0.01) -> Odometry:
    result = deepcopy(msg); result.pose.covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28): result.pose.covariance[index] = 1e6
    result.pose.covariance[35] = max(1e-6, variance); return result


class GlobalStationaryGates(Node):
    def __init__(self) -> None:
        super().__init__("global_stationary_gates")
        for name, value in {"odom_topic":"/odometry/local", "imu_topic":"/imu/data", "drive_telemetry_topic":"/controller/drive_telemetry", "stationary_speed_threshold_mps":0.03, "drive_telemetry_timeout_s":0.5, "yaw_variance_rad2":0.01}.items(): self.declare_parameter(name, value)
        p = lambda n: self.get_parameter(n).value
        self.threshold, self.timeout, self.yaw_variance = float(p("stationary_speed_threshold_mps")), float(p("drive_telemetry_timeout_s")), float(p("yaw_variance_rad2"))
        self.telemetry = None; self.telemetry_at = None
        self.odom_pub = self.create_publisher(Odometry, "/odometry/local_global", 10); self.yaw_pub = self.create_publisher(Odometry, "/odometry/local_yaw_hold", 10); self.imu_pub = self.create_publisher(Imu, "/imu/data_global", 10)
        self.create_subscription(Odometry, str(p("odom_topic")), self.on_odom, 10); self.create_subscription(Imu, str(p("imu_topic")), self.on_imu, qos_profile_sensor_data); self.create_subscription(DriveTelemetry, str(p("drive_telemetry_topic")), self.on_telemetry, 10)
    def on_telemetry(self, msg: DriveTelemetry) -> None: self.telemetry, self.telemetry_at = msg, time.monotonic()
    def active(self) -> bool:
        return self.telemetry is not None and self.telemetry_at is not None and time.monotonic() - self.telemetry_at <= self.timeout and is_stationary(self.telemetry.speed_mps_measured, self.telemetry.fresh, self.telemetry.speed_valid, self.threshold)
    def on_odom(self, msg: Odometry) -> None:
        self.odom_pub.publish(zero_twist(msg) if self.active() else msg)
        q = msg.pose.pose.orientation
        if self.active() and all(math.isfinite(v) for v in (q.x, q.y, q.z, q.w)) and q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w > 1e-9: self.yaw_pub.publish(yaw_only(msg, self.yaw_variance))
    def on_imu(self, msg: Imu) -> None: self.imu_pub.publish(zero_angular_velocity(msg) if self.active() else msg)


def main(args=None) -> None:
    rclpy.init(args=args); node = GlobalStationaryGates()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
