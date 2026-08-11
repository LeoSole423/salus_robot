"""Local planar odometry derived from the controller's measured telemetry."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from salus_interfaces.msg import DriveTelemetry


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle into the interval (-pi, pi]."""
    while angle_rad <= -math.pi:
        angle_rad += 2.0 * math.pi
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    return angle_rad


def compute_yaw_rate(speed_mps: float, steer_rad: float, wheelbase_m: float) -> float:
    """Return the yaw rate of an Ackermann vehicle, safely handling bad input."""
    if not math.isfinite(speed_mps) or not math.isfinite(steer_rad):
        return 0.0
    if abs(wheelbase_m) < 1.0e-6:
        return 0.0
    return float(speed_mps) * math.tan(float(steer_rad)) / float(wheelbase_m)


def apply_measured_steer_sign(steer_deg: float, invert_sign: bool) -> float:
    """Apply the documented hardware/simulation steering sign convention."""
    if not math.isfinite(steer_deg):
        return 0.0
    return -float(steer_deg) if bool(invert_sign) else float(steer_deg)


def integrate_planar(
    x_m: float, y_m: float, yaw_rad: float, speed_mps: float, yaw_rate_rps: float, dt_s: float
) -> tuple[float, float, float]:
    """Integrate one planar motion sample using the midpoint heading."""
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        return (float(x_m), float(y_m), float(yaw_rad))
    mid_yaw = float(yaw_rad) + 0.5 * float(yaw_rate_rps) * float(dt_s)
    return (
        float(x_m) + float(speed_mps) * math.cos(mid_yaw) * float(dt_s),
        float(y_m) + float(speed_mps) * math.sin(mid_yaw) * float(dt_s),
        normalize_angle(float(yaw_rad) + float(yaw_rate_rps) * float(dt_s)),
    )


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    quat = Quaternion()
    quat.w = math.cos(0.5 * float(yaw_rad))
    quat.z = math.sin(0.5 * float(yaw_rad))
    return quat


def diag_covariance(x_value: float, y_value: float, yaw_value: float) -> list[float]:
    covariance = [0.0] * 36
    covariance[0], covariance[7], covariance[35] = float(x_value), float(y_value), float(yaw_value)
    covariance[14] = covariance[21] = covariance[28] = 1.0e6
    return covariance


def stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


class AckermannOdometryNode(Node):
    """Publish wheel odometry without publishing TF; the EKF owns that TF."""

    def __init__(self) -> None:
        super().__init__("ackermann_odometry")
        self.declare_parameter("telemetry_topic", "/controller/drive_telemetry")
        self.declare_parameter("odom_topic", "/wheel/odometry")
        self.declare_parameter("twist_topic", "/vehicle/twist")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("wheelbase_m", 0.94)
        self.declare_parameter("steering_limit_rad", 0.5235987756)
        self.declare_parameter("invert_measured_steer_sign", False)
        self.declare_parameter("max_dt_s", 0.2)
        self.declare_parameter("require_steer_valid", False)
        self.declare_parameter("pose_covariance_xy", 0.05)
        self.declare_parameter("pose_covariance_yaw", 0.1)
        self.declare_parameter("twist_covariance_vx", 0.05)
        self.declare_parameter("twist_covariance_vy", 0.01)
        self.declare_parameter("twist_covariance_yaw_rate", 0.1)

        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._wheelbase_m = max(1.0e-6, float(self.get_parameter("wheelbase_m").value))
        self._steering_limit_rad = abs(float(self.get_parameter("steering_limit_rad").value))
        self._invert_measured_steer_sign = bool(self.get_parameter("invert_measured_steer_sign").value)
        self._max_dt_s = max(0.01, float(self.get_parameter("max_dt_s").value))
        self._require_steer_valid = bool(self.get_parameter("require_steer_valid").value)
        self._pose_covariance_xy = float(self.get_parameter("pose_covariance_xy").value)
        self._pose_covariance_yaw = float(self.get_parameter("pose_covariance_yaw").value)
        self._twist_covariance_vx = float(self.get_parameter("twist_covariance_vx").value)
        self._twist_covariance_vy = float(self.get_parameter("twist_covariance_vy").value)
        self._twist_covariance_yaw_rate = float(self.get_parameter("twist_covariance_yaw_rate").value)

        telemetry_topic = str(self.get_parameter("telemetry_topic").value)
        self._odom_pub = self.create_publisher(Odometry, str(self.get_parameter("odom_topic").value), 10)
        self._twist_pub = self.create_publisher(TwistWithCovarianceStamped, str(self.get_parameter("twist_topic").value), 10)
        self.create_subscription(DriveTelemetry, telemetry_topic, self._on_telemetry, 10)
        self._x_m = self._y_m = self._yaw_rad = 0.0
        self._last_stamp_s: Optional[float] = None

    def _on_telemetry(self, msg: DriveTelemetry) -> None:
        if not bool(msg.speed_valid) or (self._require_steer_valid and not bool(msg.steer_valid)):
            return
        stamp_s = stamp_to_seconds(msg.stamp)
        if stamp_s <= 0.0:
            stamp_s = self.get_clock().now().nanoseconds / 1.0e9
        speed_mps = abs(float(msg.speed_mps_measured))
        if bool(msg.reverse_requested):
            speed_mps = -speed_mps
        steer_deg = float(msg.steer_deg_measured) if bool(msg.steer_valid) else 0.0
        steer_rad = math.radians(apply_measured_steer_sign(steer_deg, self._invert_measured_steer_sign))
        steer_rad = max(-self._steering_limit_rad, min(self._steering_limit_rad, steer_rad))
        yaw_rate_rps = compute_yaw_rate(speed_mps, steer_rad, self._wheelbase_m)
        if bool(msg.fresh):
            if self._last_stamp_s is not None:
                dt_s = stamp_s - self._last_stamp_s
                if 0.0 < dt_s <= self._max_dt_s:
                    self._x_m, self._y_m, self._yaw_rad = integrate_planar(
                        self._x_m, self._y_m, self._yaw_rad, speed_mps, yaw_rate_rps, dt_s
                    )
            self._last_stamp_s = stamp_s
        else:
            speed_mps = yaw_rate_rps = 0.0
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = msg.stamp, self._odom_frame, self._base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = self._x_m, self._y_m
        odom.pose.pose.orientation = quaternion_from_yaw(self._yaw_rad)
        odom.pose.covariance = diag_covariance(self._pose_covariance_xy, self._pose_covariance_xy, self._pose_covariance_yaw)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = speed_mps, yaw_rate_rps
        odom.twist.covariance = diag_covariance(self._twist_covariance_vx, self._twist_covariance_vy, self._twist_covariance_yaw_rate)
        self._odom_pub.publish(odom)
        twist = TwistWithCovarianceStamped()
        twist.header, twist.twist = odom.header, odom.twist
        self._twist_pub.publish(twist)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
