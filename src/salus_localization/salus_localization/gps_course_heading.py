"""GPS course-heading estimator with explicit motion and RTK gates."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
import time
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
        self.speed = self.yaw_rate = 0.0
        self.steer: Optional[float] = None
        self.steer_valid = False
        self.rtk_status = ""
        self.rtk_at_monotonic: Optional[float] = None
        self.output = self.create_publisher(Imu, str(p("output_topic")), 10)
        self.debug = self.create_publisher(String, str(p("debug_topic")), 10)
        self._pending_output: Optional[Imu] = None
        self._pending_output_stamp_s: Optional[float] = None
        self._delivery_timer = self.create_timer(0.1, self._flush_pending_output)
        self.create_subscription(
            NavSatFix, str(p("gps_topic")), self.on_fix, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, str(p("odom_topic")), self.on_odom, 10)
        self.create_subscription(
            DriveTelemetry, str(p("drive_telemetry_topic")), self.on_drive, 10
        )
        self.create_subscription(String, str(p("rtk_status_topic")), self.on_rtk, 10)
        # Course-over-ground gains new information only when a new GNSS fix
        # arrives. Evaluate on that causal boundary instead of depending on a
        # periodic timer that can be starved by high-rate odometry callbacks.
    
    def now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def on_fix(self, msg: NavSatFix) -> None:
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp_s <= 0.0:
            stamp_s = self.now()
            output_stamp = self.get_clock().now().to_msg()
        else:
            output_stamp = msg.header.stamp
        self.estimator.add_fix(msg.latitude, msg.longitude, stamp_s)
        self.publish(now_s=stamp_s, output_stamp=output_stamp)

    def on_odom(self, msg: Odometry) -> None:
        self.speed = msg.twist.twist.linear.x
        self.yaw_rate = msg.twist.twist.angular.z

    def on_drive(self, msg: DriveTelemetry) -> None:
        self.steer_valid = bool(msg.fresh and msg.steer_valid)
        self.steer = msg.steer_deg_measured

    def on_rtk(self, msg: String) -> None:
        self.rtk_status = msg.data
        self.rtk_at_monotonic = time.monotonic()

    def publish(self, *, now_s: Optional[float] = None, output_stamp=None) -> None:
        now = self.now() if now_s is None else float(now_s)
        p = lambda n: self.get_parameter(n).value
        rtk_age_s = (
            None
            if self.rtk_at_monotonic is None
            else max(0.0, time.monotonic() - self.rtk_at_monotonic)
        )
        rtk_valid = (
            not bool(p("require_rtk"))
            or (
                rtk_age_s is not None
                and rtk_age_s <= float(p("rtk_status_max_age_s"))
                and normalize_rtk_status_label(self.rtk_status)
                in ("rtk_fixed", "rtk_fix")
            )
        )
        estimate = (
            self.estimator.estimate(
                now_s=now,
                speed_mps=self.speed,
                steer_deg=self.steer,
                steer_valid=self.steer_valid,
                yaw_rate_rps=self.yaw_rate,
            )
            if rtk_valid
            else HeadingEstimate(
                False,
                "rtk_status_rejected_or_stale",
                None,
                0.0,
                self.speed,
                None,
            )
        )
        debug_payload = {
            "valid": estimate.valid,
            "reason": estimate.reason,
            "distance_m": estimate.distance_m,
            "speed_mps": estimate.speed_mps,
            "sample_dt_s": estimate.sample_dt_s,
            "steer_valid": self.steer_valid,
            "steer_deg": self.steer,
            "yaw_rate_rps": self.yaw_rate,
            "rtk_valid": rtk_valid,
            "rtk_age_s": rtk_age_s,
        }
        if estimate.valid and estimate.yaw_rad is not None:
            msg = Imu()
            stamp = (
                self.get_clock().now().to_msg()
                if output_stamp is None
                else output_stamp
            )
            msg.header.stamp.sec = int(stamp.sec)
            msg.header.stamp.nanosec = int(stamp.nanosec)
            msg.header.frame_id = str(p("base_frame"))
            msg.orientation.z = math.sin(estimate.yaw_rad / 2)
            msg.orientation.w = math.cos(estimate.yaw_rad / 2)
            msg.orientation_covariance[8] = (
                0.05 if estimate.reason == "ok" else 0.2
            )
            self._deliver_or_buffer(msg, now_s=now)
        debug_payload["output_subscribers"] = self.output.get_subscription_count()
        debug_payload["pending_output"] = self._pending_output is not None
        self.debug.publish(String(data=json.dumps(debug_payload, sort_keys=True)))

    def _deliver_or_buffer(self, message: Imu, *, now_s: float) -> None:
        if self.output.get_subscription_count() > 0:
            self.output.publish(message)
            self._pending_output = None
            self._pending_output_stamp_s = None
            return
        self._pending_output = deepcopy(message)
        self._pending_output_stamp_s = float(now_s)

    def _flush_pending_output(self) -> None:
        if self._pending_output is None or self._pending_output_stamp_s is None:
            return
        if self.output.get_subscription_count() <= 0:
            return
        max_age_s = float(self.get_parameter("max_fix_age_s").value)
        if max(0.0, self.now() - self._pending_output_stamp_s) > max_age_s:
            self._pending_output = None
            self._pending_output_stamp_s = None
            return
        self.output.publish(self._pending_output)
        self._pending_output = None
        self._pending_output_stamp_s = None


def main(args=None) -> None:
    rclpy.init(args=args); node = GpsCourseHeading()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
