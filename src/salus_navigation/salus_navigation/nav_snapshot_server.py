"""ROS adapter for the navigation snapshot contract defined in ADR 0004."""

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from salus_interfaces.msg import NavSnapshotLayers
from salus_interfaces.srv import GetNavSnapshot
from salus_navigation.snapshot_renderer import Grid, Polyline, SnapshotScene, Transform2D, render


@dataclass(frozen=True)
class Cached:
    message: Any
    first_received_monotonic: float


class NavSnapshotServer(Node):
    """Owns ROS I/O only; rendering is delegated to ``snapshot_renderer``."""

    def __init__(self) -> None:
        super().__init__("nav_snapshot_server")
        defaults = {
            "get_snapshot_service": "/nav_snapshot_server/get_nav_snapshot",
            "local_costmap_topic": "/local_costmap/costmap",
            "global_costmap_topic": "/global_costmap/costmap",
            "keepout_mask_topic": "/keepout_filter_mask",
            "local_footprint_topic": "/local_costmap/published_footprint",
            "stop_zone_topic": "/stop_zone_raw",
            "collision_polygons_topic": "/collision_monitor/polygons",
            "scan_topic": "/scan_clean",
            "plan_topic": "/plan",
            "base_frame": "base_footprint",
            "snapshot_extent_m": 30.0,
            "snapshot_size_px": 512,
            "snapshot_global_inset_px": 160,
            "snapshot_timeout_ms": 500,
            "tf_timeout_s": 0.2,
            "local_costmap_max_age_s": 2.0,
            "dynamic_layer_max_age_s": 2.0,
            "startup_grace_s": 5.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._parameter = {name: self.get_parameter(name).value for name in defaults}
        self._lock = threading.Lock()
        self._cache: Dict[str, Cached] = {}
        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._listener = TransformListener(self._tf, self)
        grid_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, self._parameter["local_costmap_topic"], lambda msg: self._cache_message("local", msg), grid_qos)
        self.create_subscription(OccupancyGrid, self._parameter["global_costmap_topic"], lambda msg: self._cache_message("global", msg), grid_qos)
        self.create_subscription(OccupancyGrid, self._parameter["keepout_mask_topic"], lambda msg: self._cache_message("keepout", msg), grid_qos)
        self.create_subscription(PolygonStamped, self._parameter["local_footprint_topic"], lambda msg: self._cache_message("footprint", msg), 10)
        self.create_subscription(PolygonStamped, self._parameter["stop_zone_topic"], lambda msg: self._cache_message("stop_zone", msg), 10)
        self.create_subscription(MarkerArray, self._parameter["collision_polygons_topic"], lambda msg: self._cache_message("collision", msg), 10)
        self.create_subscription(LaserScan, self._parameter["scan_topic"], lambda msg: self._cache_message("scan", msg), qos_profile_sensor_data)
        self.create_subscription(Path, self._parameter["plan_topic"], lambda msg: self._cache_message("plan", msg), 10)
        self.create_service(GetNavSnapshot, self._parameter["get_snapshot_service"], self._on_snapshot)

    def _cache_message(self, key: str, message: Any) -> None:
        with self._lock:
            previous = self._cache.get(key)
            first = previous.first_received_monotonic if previous else time.monotonic()
            self._cache[key] = Cached(message, first)

    def _copy_cache(self) -> Dict[str, Cached]:
        with self._lock:
            return dict(self._cache)

    def _stamp_age_ok(self, cached: Cached, required: bool) -> bool:
        header = getattr(cached.message, "header", None)
        stamp = getattr(header, "stamp", None)
        stamp_ns = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        now_ns = self.get_clock().now().nanoseconds
        if stamp_ns <= 0 or now_ns <= 0:
            return time.monotonic() - cached.first_received_monotonic <= float(self._parameter["startup_grace_s"])
        max_age = float(self._parameter["local_costmap_max_age_s"] if required else self._parameter["dynamic_layer_max_age_s"])
        return now_ns - stamp_ns <= int(max_age * 1_000_000_000)

    @staticmethod
    def _grid(message: OccupancyGrid) -> Optional[Grid]:
        info = message.info
        if info.width <= 0 or info.height <= 0 or info.resolution <= 0.0 or len(message.data) != info.width * info.height:
            return None
        return Grid(message.header.frame_id, float(info.resolution),
                    (float(info.origin.position.x), float(info.origin.position.y)),
                    int(info.width), int(info.height), tuple(int(value) for value in message.data))

    def _transform(self, target: str, source: str, stamp: TimeMsg) -> Optional[Transform2D]:
        if not target or not source or target == source:
            return Transform2D(source, target, 0.0, 0.0, 0.0)
        try:
            transform = self._tf.lookup_transform(target, source, Time.from_msg(stamp), Duration(seconds=float(self._parameter["tf_timeout_s"])))
        except TransformException:
            return None
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = transform.transform.translation
        return Transform2D(source, target, float(t.x), float(t.y), yaw)

    def _polyline(self, message: PolygonStamped, color: Tuple[int, int, int]) -> Optional[Polyline]:
        points = tuple((float(point.x), float(point.y)) for point in message.polygon.points)
        return Polyline(message.header.frame_id or str(self._parameter["base_frame"]), points, color, 2, True) if len(points) >= 3 else None

    def _scan(self, message: LaserScan) -> Optional[Polyline]:
        points = []
        for index, distance in enumerate(message.ranges):
            if not math.isfinite(distance) or distance < message.range_min or distance > message.range_max:
                continue
            angle = message.angle_min + index * message.angle_increment
            points.append((float(distance * math.cos(angle)), float(distance * math.sin(angle))))
        return Polyline(message.header.frame_id or str(self._parameter["base_frame"]), tuple(points), (0, 80, 255), 1, False) if points else None

    def _path(self, message: Path) -> Optional[Polyline]:
        points = tuple((float(pose.pose.position.x), float(pose.pose.position.y)) for pose in message.poses)
        frame = message.header.frame_id or (message.poses[0].header.frame_id if message.poses else "") or str(self._parameter["base_frame"])
        return Polyline(frame, points, (64, 255, 64), 2, False) if len(points) >= 2 else None

    def _collision(self, message: MarkerArray) -> Tuple[Polyline, ...]:
        result = []
        for marker in message.markers:
            if len(marker.points) < 2:
                continue
            points = tuple((float(point.x + marker.pose.position.x), float(point.y + marker.pose.position.y)) for point in marker.points)
            color = (int(marker.color.b * 255), int(marker.color.g * 255), int(marker.color.r * 255))
            result.append(Polyline(marker.header.frame_id or str(self._parameter["base_frame"]), points, color if any(color) else (0, 200, 255), 2, marker.type == Marker.LINE_STRIP))
        return tuple(result)

    def _layers(self, values: Dict[str, bool]) -> NavSnapshotLayers:
        response = NavSnapshotLayers()
        for field, value in values.items():
            setattr(response, field, bool(value))
        return response

    def _failure(self, response: GetNavSnapshot.Response, error: str, frame_id: str = "") -> GetNavSnapshot.Response:
        response.ok = False
        response.error = error
        response.mime = ""
        response.width = response.height = 0
        response.frame_id = frame_id
        response.stamp = self.get_clock().now().to_msg()
        response.layers = NavSnapshotLayers()
        response.image_png = []
        return response

    def _on_snapshot(self, _request: GetNavSnapshot.Request, response: GetNavSnapshot.Response) -> GetNavSnapshot.Response:
        started = time.monotonic()
        cache = self._copy_cache()
        local_cached = cache.get("local")
        if local_cached is None:
            return self._failure(response, "MISSING_LOCAL_COSTMAP: no local costmap received")
        local = self._grid(local_cached.message)
        if local is None:
            return self._failure(response, "INVALID_LOCAL_COSTMAP: invalid grid metadata", local_cached.message.header.frame_id)
        if not self._stamp_age_ok(local_cached, True):
            return self._failure(response, "STALE_LOCAL_COSTMAP: local costmap exceeded age limit", local.frame_id)
        base = str(self._parameter["base_frame"])
        robot = self._transform(local.frame_id, base, local_cached.message.header.stamp)
        if robot is None:
            return self._failure(response, "MISSING_LOCAL_TF: base_footprint transform unavailable", local.frame_id)
        transforms = [robot]
        dynamic = lambda key: cache.get(key) if key in cache and self._stamp_age_ok(cache[key], False) else None
        global_cached, keepout_cached = dynamic("global"), cache.get("keepout")
        global_grid = self._grid(global_cached.message) if global_cached else None
        keepout_grid = self._grid(keepout_cached.message) if keepout_cached else None
        for candidate in (global_cached, keepout_cached, dynamic("footprint"), dynamic("stop_zone"), dynamic("collision"), dynamic("scan"), dynamic("plan")):
            if candidate is None:
                continue
            header = getattr(candidate.message, "header", None)
            frame = getattr(header, "frame_id", "") or local.frame_id
            stamp = getattr(header, "stamp", local_cached.message.header.stamp)
            transform = self._transform(local.frame_id, frame, stamp)
            if transform is not None:
                transforms.append(transform)
        footprint_cached, stop_cached = dynamic("footprint"), dynamic("stop_zone")
        scan_cached, plan_cached, collision_cached = dynamic("scan"), dynamic("plan"), dynamic("collision")
        scene = SnapshotScene(
            local, (robot.x, robot.y), max(5.0, float(self._parameter["snapshot_extent_m"])),
            max(128, min(1024, int(self._parameter["snapshot_size_px"]))),
            max(80, min(int(self._parameter["snapshot_size_px"]) // 2, int(self._parameter["snapshot_global_inset_px"]))),
            keepout_grid, global_grid,
            self._polyline(footprint_cached.message, (0, 255, 0)) if footprint_cached else None,
            self._polyline(stop_cached.message, (0, 0, 255)) if stop_cached else None,
            self._collision(collision_cached.message) if collision_cached else (),
            self._scan(scan_cached.message) if scan_cached else None,
            self._path(plan_cached.message) if plan_cached else None,
            tuple(transforms),
        )
        try:
            rendered = render(scene)
        except RuntimeError as exc:
            return self._failure(response, str(exc), local.frame_id)
        except Exception as exc:  # diagnostics retain the public stable prefix
            self.get_logger().error(f"snapshot internal error: {exc}")
            return self._failure(response, f"INTERNAL_ERROR: {exc}", local.frame_id)
        response.ok = True
        response.error = ""
        response.mime = "image/png"
        response.width, response.height = rendered.width, rendered.height
        response.frame_id = rendered.frame_id
        response.stamp = self.get_clock().now().to_msg()
        response.layers = self._layers(rendered.layers)
        response.image_png = list(rendered.png)
        elapsed_ms = (time.monotonic() - started) * 1000.0
        if elapsed_ms > float(self._parameter["snapshot_timeout_ms"]):
            self.get_logger().warning(f"snapshot generation exceeded target: {elapsed_ms:.1f} ms")
        return response


def main() -> None:
    rclpy.init()
    node = NavSnapshotServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
