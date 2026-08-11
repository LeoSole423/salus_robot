"""GPS course-heading estimator with explicit motion and RTK gates."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from typing import Deque, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from salus_interfaces.msg import DriveTelemetry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String


@dataclass(frozen=True)
class HeadingEstimate:
    valid: bool
    reason: str
    yaw_rad: Optional[float]
    distance_m: float
    speed_mps: float
    sample_dt_s: Optional[float]


def heading_from_fixes(previous: NavSatFix, current: NavSatFix) -> tuple[float, float]:
    north = (current.latitude - previous.latitude) * 111_320.0
    east = (current.longitude - previous.longitude) * 111_320.0 * math.cos(math.radians(current.latitude))
    return math.atan2(north, east), math.hypot(north, east)


class CourseHeadingEstimator:
    def __init__(self, *, min_distance_m: float = 2.0, min_speed_mps: float = 0.8,
                 max_abs_steer_deg: float = 3.0, max_abs_yaw_rate_rps: float = 0.05,
                 max_fix_age_s: float = 0.5, max_sample_dt_s: float = 2.5,
                 invalid_hold_s: float = 0.8) -> None:
        self.min_distance_m, self.min_speed_mps = min_distance_m, min_speed_mps
        self.max_abs_steer_deg, self.max_abs_yaw_rate_rps = max_abs_steer_deg, max_abs_yaw_rate_rps
        self.max_fix_age_s, self.max_sample_dt_s, self.invalid_hold_s = max_fix_age_s, max_sample_dt_s, invalid_hold_s
        self.fixes: Deque[tuple[float, float, float]] = deque()
        self.last_valid: Optional[HeadingEstimate] = None
        self.last_valid_at: Optional[float] = None

    def add_fix(self, lat: float, lon: float, stamp_s: float) -> None:
        if all(math.isfinite(value) for value in (lat, lon, stamp_s)):
            self.fixes.append((lat, lon, stamp_s))
        while self.fixes and self.fixes[0][2] < stamp_s - 12.0:
            self.fixes.popleft()

    def estimate(self, *, now_s: float, speed_mps: float, steer_deg: Optional[float],
                 steer_valid: bool, yaw_rate_rps: float) -> HeadingEstimate:
        def invalid(reason: str, hold: bool = False) -> HeadingEstimate:
            if hold and self.last_valid and self.last_valid_at is not None and now_s - self.last_valid_at <= self.invalid_hold_s:
                return HeadingEstimate(True, "hold_" + reason, self.last_valid.yaw_rad,
                                       self.last_valid.distance_m, speed_mps, self.last_valid.sample_dt_s)
            return HeadingEstimate(False, reason, None, 0.0, speed_mps, None)
        if not self.fixes:
            return invalid("no_fix")
        latest = self.fixes[-1]
        if now_s - latest[2] > self.max_fix_age_s:
            return invalid("stale_fix")
        if not math.isfinite(speed_mps) or speed_mps < self.min_speed_mps:
            return invalid("speed_below_threshold")
        if not steer_valid or steer_deg is None or not math.isfinite(steer_deg):
            return invalid("steer_invalid")
        if abs(steer_deg) > self.max_abs_steer_deg:
            return invalid("steer_too_high", True)
        if not math.isfinite(yaw_rate_rps) or abs(yaw_rate_rps) > self.max_abs_yaw_rate_rps:
            return invalid("yaw_rate_too_high", True)
        for candidate in reversed(list(self.fixes)[:-1]):
            dt = latest[2] - candidate[2]
            if dt > self.max_sample_dt_s:
                break
            north = (latest[0] - candidate[0]) * 111_320.0
            east = (latest[1] - candidate[1]) * 111_320.0 * math.cos(math.radians(candidate[0]))
            distance = math.hypot(north, east)
            if distance >= self.min_distance_m:
                result = HeadingEstimate(True, "ok", math.atan2(north, east), distance, speed_mps, dt)
                self.last_valid, self.last_valid_at = result, now_s
                return result
        return invalid("distance_below_threshold")


def normalize_rtk_status_label(value: str) -> str:
    return "_".join(str(value).strip().lower().replace("-", "_").split())


class GpsCourseHeading(Node):
    def __init__(self) -> None:
        super().__init__("gps_course_heading")
        for name, value in {"gps_topic":"/gps/fix", "odom_topic":"/odometry/local", "drive_telemetry_topic":"/controller/drive_telemetry", "rtk_status_topic":"/gps/rtk_status", "output_topic":"/gps/course_heading", "debug_topic":"/gps/course_heading/debug", "base_frame":"base_footprint", "min_distance_m":2.0, "min_speed_mps":0.8, "max_abs_steer_deg":3.0, "max_abs_yaw_rate_rps":0.05, "max_fix_age_s":0.5, "max_sample_dt_s":2.5, "invalid_hold_s":0.8, "require_rtk":True, "rtk_status_max_age_s":2.5}.items():
            self.declare_parameter(name, value)
        p = lambda n: self.get_parameter(n).value
        self.estimator = CourseHeadingEstimator(min_distance_m=float(p("min_distance_m")), min_speed_mps=float(p("min_speed_mps")), max_abs_steer_deg=float(p("max_abs_steer_deg")), max_abs_yaw_rate_rps=float(p("max_abs_yaw_rate_rps")), max_fix_age_s=float(p("max_fix_age_s")), max_sample_dt_s=float(p("max_sample_dt_s")), invalid_hold_s=float(p("invalid_hold_s")))
        self.speed = self.yaw_rate = 0.0; self.steer: Optional[float] = None; self.steer_valid = False
        self.rtk_status, self.rtk_at = "", None
        self.output = self.create_publisher(Imu, str(p("output_topic")), 10); self.debug = self.create_publisher(String, str(p("debug_topic")), 10)
        self.create_subscription(NavSatFix, str(p("gps_topic")), self.on_fix, qos_profile_sensor_data)
        self.create_subscription(Odometry, str(p("odom_topic")), self.on_odom, 10)
        self.create_subscription(DriveTelemetry, str(p("drive_telemetry_topic")), self.on_drive, 10)
        self.create_subscription(String, str(p("rtk_status_topic")), self.on_rtk, 10)
        self.create_timer(0.2, self.publish)

    def now(self) -> float: return self.get_clock().now().nanoseconds / 1e9
    def on_fix(self, msg: NavSatFix) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.estimator.add_fix(msg.latitude, msg.longitude, stamp if stamp > 0 else self.now())
    def on_odom(self, msg: Odometry) -> None: self.speed, self.yaw_rate = msg.twist.twist.linear.x, msg.twist.twist.angular.z
    def on_drive(self, msg: DriveTelemetry) -> None: self.steer_valid, self.steer = bool(msg.fresh and msg.steer_valid), msg.steer_deg_measured
    def on_rtk(self, msg: String) -> None: self.rtk_status, self.rtk_at = msg.data, self.now()
    def publish(self) -> None:
        now = self.now(); p = lambda n: self.get_parameter(n).value
        rtk_valid = not bool(p("require_rtk")) or (self.rtk_at is not None and now - self.rtk_at <= float(p("rtk_status_max_age_s")) and normalize_rtk_status_label(self.rtk_status) in ("rtk_fixed", "rtk_fix"))
        estimate = self.estimator.estimate(now_s=now, speed_mps=self.speed, steer_deg=self.steer, steer_valid=self.steer_valid, yaw_rate_rps=self.yaw_rate) if rtk_valid else HeadingEstimate(False, "rtk_status_rejected_or_stale", None, 0.0, self.speed, None)
        self.debug.publish(String(data=json.dumps({"valid":estimate.valid, "reason":estimate.reason, "distance_m":estimate.distance_m, "speed_mps":estimate.speed_mps, "sample_dt_s":estimate.sample_dt_s}, sort_keys=True)))
        if estimate.valid and estimate.yaw_rad is not None:
            msg = Imu(); msg.header.stamp = self.get_clock().now().to_msg(); msg.header.frame_id = str(p("base_frame")); msg.orientation.z = math.sin(estimate.yaw_rad / 2); msg.orientation.w = math.cos(estimate.yaw_rad / 2); msg.orientation_covariance[8] = 0.05 if estimate.reason == "ok" else 0.2; self.output.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args); node = GpsCourseHeading()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
