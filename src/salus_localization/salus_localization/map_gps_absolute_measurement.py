"""Publish absolute GPS position in the fixed simulation map frame."""
from __future__ import annotations

import math
from collections import deque
from copy import deepcopy
import json
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from .gnss_antenna_compensation import (MapHeading, MapHeadingHistory, compensate_antenna, rotate_covariance)
from robot_localization.srv import FromLL
from sensor_msgs.msg import NavSatFix


def project_fix(
    latitude: float,
    longitude: float,
    datum_lat: float,
    datum_lon: float,
    datum_yaw_deg: float = 0.0,
) -> tuple[float, float]:
    """Project WGS84 into the datum-aligned map frame used by simulation."""
    north = (latitude - datum_lat) * 111_320.0
    east = (longitude - datum_lon) * 111_320.0 * math.cos(math.radians(datum_lat))
    yaw_rad = math.radians(datum_yaw_deg)
    return (
        math.cos(yaw_rad) * east + math.sin(yaw_rad) * north,
        -math.sin(yaw_rad) * east + math.cos(yaw_rad) * north,
    )


class MapGpsAbsoluteMeasurement(Node):
    def __init__(self) -> None:
        super().__init__("map_gps_absolute_measurement")
        defaults = {
            "gps_topic": "/gps/fix", "output_topic": "/gps/odometry_map",
            "datum_lat": -31.4858037, "datum_lon": -64.2410570,
            "datum_yaw_deg": 0.0, "covariance_xy": 0.05,
            "map_odom_topic": "/odometry/global", "map_frame": "map",
            "base_frame": "base_footprint", "orientation_topic": "/localization/orientation",
            "max_fix_age_s": 0.5, "max_odom_age_s": 0.5,
            "max_heading_gap_s": 0.2, "future_tolerance_s": 0.05,
            "compensate_antenna_position": True,
            "mount_variance_m2": 0.0, "debug_topic": "/gps/odometry_map/debug",
        }
        for name,value in defaults.items():
            self.declare_parameter(name,value)
        p = lambda name: self.get_parameter(name).value
        for name in ("covariance_xy","max_fix_age_s","max_odom_age_s","max_heading_gap_s"):
            if not math.isfinite(float(p(name))) or float(p(name)) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ("future_tolerance_s","mount_variance_m2"):
            if not math.isfinite(float(p(name))) or float(p(name)) < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        self.map_frame,self.base_frame = str(p("map_frame")),str(p("base_frame"))
        if not self.map_frame.strip() or not self.base_frame.strip():
            raise ValueError("map_frame and base_frame must be nonempty")
        self.headings = MapHeadingHistory(float(p("max_heading_gap_s")))
        self.pending = deque(maxlen=20)
        self.absolute_heading_since = None
        self.last_fix_stamp = None
        self.last_now = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        self.publisher = self.create_publisher(Odometry,str(p("output_topic")),10)
        self.debug = self.create_publisher(String,str(p("debug_topic")),10)
        self.create_subscription(NavSatFix,str(p("gps_topic")),self.on_fix,10)
        self.create_subscription(Odometry,str(p("map_odom_topic")),self.on_map_odom,10)
        self.create_subscription(Imu,str(p("orientation_topic")),self.on_absolute_heading,qos_profile_sensor_data)
        self.create_timer(0.05,self.flush)
        # /fromLL transforms a geographical point, not an antenna measurement.
        self.create_service(FromLL,"/fromLL",self.on_from_ll)

    def now_s(self):
        now = self.get_clock().now().nanoseconds/1e9
        if self.last_now is not None and now < self.last_now:
            self.pending.clear()
            self.headings.samples.clear()
            self.absolute_heading_since = None
            self.last_fix_stamp = None
        self.last_now = now
        return now

    @staticmethod
    def stamp_s(stamp):
        return stamp.sec+stamp.nanosec/1e9

    def fresh(self, stamp, now, parameter):
        return -float(self.get_parameter("future_tolerance_s").value) <= now-stamp <= float(self.get_parameter(parameter).value)

    def report(self, reason, fix=None, **metrics):
        self.debug.publish(String(data=json.dumps(dict(
            valid=reason=="ok", reason=reason,
            stamp_s=self.stamp_s(fix.header.stamp) if fix is not None else None,
            **metrics),sort_keys=True)))

    def on_absolute_heading(self, msg):
        now = self.now_s()
        stamp = self.stamp_s(msg.header.stamp)
        q = msg.orientation
        values = (q.x,q.y,q.z,q.w,*msg.orientation_covariance)
        if (stamp <= 0 or msg.header.frame_id != self.base_frame
                or not self.fresh(stamp,now,"max_odom_age_s")
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)>1e-3
                or msg.orientation_covariance[0]<0 or msg.orientation_covariance[4]<0 or msg.orientation_covariance[8]<=0):
            return
        # This latches initialization only. The fresh map estimate below follows
        # the maneuver continuously when absolute heading is intentionally absent.
        if self.absolute_heading_since is None:
            self.absolute_heading_since = stamp
        self.flush()

    def on_map_odom(self, msg):
        now = self.now_s()
        stamp = self.stamp_s(msg.header.stamp)
        q = msg.pose.pose.orientation
        values = (q.x,q.y,q.z,q.w,msg.pose.covariance[35])
        if (msg.header.frame_id != self.map_frame or msg.child_frame_id != self.base_frame
                or not self.fresh(stamp,now,"max_odom_age_s")
                or not all(math.isfinite(v) for v in values)
                or abs(sum(v*v for v in values[:4])-1)>1e-3):
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        self.headings.add(MapHeading(stamp,yaw,msg.pose.covariance[35]))
        self.flush()

    def on_fix(self, fix):
        now = self.now_s()
        stamp = self.stamp_s(fix.header.stamp)
        if (stamp <= 0 or not math.isfinite(fix.latitude) or not math.isfinite(fix.longitude)
                or not -90<=fix.latitude<=90 or not -180<=fix.longitude<=180
                or not fix.header.frame_id or fix.status.status < 0):
            self.report("invalid_fix",fix)
            return
        if not self.fresh(stamp,now,"max_fix_age_s"):
            self.report("stale_or_future_fix",fix)
            return
        if self.last_fix_stamp is not None and stamp <= self.last_fix_stamp:
            self.report("non_monotonic_fix",fix)
            return
        self.last_fix_stamp = stamp
        if len(self.pending)==self.pending.maxlen:
            self.report("pending_queue_full",self.pending[0])
        self.pending.append(deepcopy(fix))
        self.flush()

    def flush(self):
        now = self.now_s()
        while self.pending:
            fix = self.pending[0]
            stamp = self.stamp_s(fix.header.stamp)
            if not self.fresh(stamp,now,"max_fix_age_s"):
                self.pending.popleft()
                self.report("stale_or_future_fix",fix)
                continue
            if self.absolute_heading_since is None or stamp < self.absolute_heading_since:
                self.pending.popleft()
                self.report("absolute_heading_uninitialized",fix)
                continue
            if not self.headings.samples or not self.fresh(self.headings.samples[-1].stamp_s,now,"max_odom_age_s"):
                self.report("map_heading_unavailable_or_stale",fix)
                return
            heading = self.headings.at(stamp)
            if heading is None:
                if self.headings.samples[-1].stamp_s < stamp:
                    return  # Wait causally for the second interpolation endpoint.
                self.pending.popleft()
                self.report("map_heading_gap",fix)
                continue
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,fix.header.frame_id,Time.from_msg(fix.header.stamp))
            except TransformException:
                self.report("antenna_transform_unavailable",fix)
                return
            lever = transform.transform.translation
            floor = float(self.get_parameter("covariance_xy").value)
            if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
                covariance = (floor,0.0,floor)
            else:
                c = fix.position_covariance
                if not all(math.isfinite(v) for v in c) or abs(c[1]-c[3])>1e-6:
                    self.pending.popleft()
                    self.report("invalid_position_covariance",fix)
                    continue
                covariance = (c[0],c[1],c[4])
            try:
                covariance = rotate_covariance(*covariance,-math.radians(float(self.get_parameter("datum_yaw_deg").value)))
                covariance = (max(covariance[0],floor),covariance[1],max(covariance[2],floor))
                x,y = project_fix(fix.latitude,fix.longitude,
                    float(self.get_parameter("datum_lat").value),float(self.get_parameter("datum_lon").value),
                    float(self.get_parameter("datum_yaw_deg").value))
                x,y,covariance = compensate_antenna(x,y,heading,lever.x,lever.y,covariance,
                    float(self.get_parameter("mount_variance_m2").value),
                    compensate_position=bool(self.get_parameter("compensate_antenna_position").value))
            except ValueError:
                self.pending.popleft()
                self.report("invalid_position_covariance_or_geometry",fix)
                continue
            msg = Odometry()
            msg.header = deepcopy(fix.header)
            msg.header.frame_id = self.map_frame
            msg.child_frame_id = self.base_frame
            msg.pose.pose.position.x,msg.pose.pose.position.y = x,y
            msg.pose.pose.orientation.w = 1.0
            c = [0.0]*36
            c[0],c[1],c[6],c[7] = covariance[0],covariance[1],covariance[1],covariance[2]
            c[14]=c[21]=c[28]=c[35]=1e6
            msg.pose.covariance = c
            self.pending.popleft()
            self.publisher.publish(msg)
            self.report("ok",fix,yaw_map_rad=heading.yaw_rad,yaw_variance_rad2=heading.variance_rad2,
                lever_x_m=lever.x,lever_y_m=lever.y,covariance_xy=list(covariance),
                compensation_enabled=bool(self.get_parameter("compensate_antenna_position").value))

    def on_from_ll(self, request: FromLL.Request, response: FromLL.Response) -> FromLL.Response:
        response.map_point.x, response.map_point.y = project_fix(
            request.ll_point.latitude,
            request.ll_point.longitude,
            float(self.get_parameter("datum_lat").value),
            float(self.get_parameter("datum_lon").value),
            float(self.get_parameter("datum_yaw_deg").value),
        )
        response.map_point.z = 0.0
        return response


def main(args=None) -> None:
    rclpy.init(args=args); node = MapGpsAbsoluteMeasurement()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
