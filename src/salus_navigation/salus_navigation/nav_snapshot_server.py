"""ROS adapter for the navigation snapshot contract defined in ADR 0004."""

from dataclasses import dataclass
import copy
import math
import threading
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PolygonStamped
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from salus_interfaces.msg import NavSnapshotLayers, ProjectedKeepoutState
from salus_interfaces.srv import GetNavSnapshot
from salus_navigation.snapshot_renderer import Grid, KeepoutPolygon, Polyline, SnapshotScene, Transform2D, render


@dataclass(frozen=True)
class Cached:
    message: Any
    first_received_monotonic: float


def is_fresh(stamp_ns: int, now_ns: int, received_monotonic: float,
             now_monotonic: float, max_age_s: float, startup_grace_s: float) -> bool:
    """Pure freshness policy from ADR 0004."""
    if stamp_ns <= 0 or now_ns <= 0:
        return now_monotonic - received_monotonic <= startup_grace_s
    age_ns = now_ns - stamp_ns
    return 0 <= age_ns <= int(max_age_s * 1_000_000_000)


def apply_grid_update(grid: OccupancyGrid, update: OccupancyGridUpdate) -> Optional[OccupancyGrid]:
    """Return a copied grid with one valid Nav2 incremental update applied."""
    width, height = int(update.width), int(update.height)
    x, y = int(update.x), int(update.y)
    if (width <= 0 or height <= 0 or x < 0 or y < 0
            or x + width > grid.info.width or y + height > grid.info.height
            or len(update.data) != width * height):
        return None
    result = copy.deepcopy(grid)
    values = list(result.data)
    for row in range(height):
        source = row * width
        target = (y + row) * int(grid.info.width) + x
        values[target:target + width] = update.data[source:source + width]
    result.data = values
    result.header.stamp = update.header.stamp
    if update.header.frame_id:
        result.header.frame_id = update.header.frame_id
    return result


class NavSnapshotServer(Node):
    """Owns ROS I/O only; rendering is delegated to ``snapshot_renderer``."""

    def __init__(self) -> None:
        super().__init__("nav_snapshot_server")
        defaults = {
            "get_snapshot_service": "/nav_snapshot_server/get_nav_snapshot",
            "local_costmap_topic": "/local_costmap/costmap",
            "local_costmap_updates_topic": "/local_costmap/costmap_updates",
            "global_costmap_topic": "/global_costmap/costmap",
            "global_costmap_updates_topic": "/global_costmap/costmap_updates",
            "projected_keepouts_topic": "/zones_manager/projected_keepouts",
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
        self._parameter["snapshot_extent_m"] = max(5.0, float(self._parameter["snapshot_extent_m"]))
        self._parameter["snapshot_size_px"] = max(128, min(1024, int(self._parameter["snapshot_size_px"])))
        self._parameter["snapshot_global_inset_px"] = max(
            32,
            min(self._parameter["snapshot_size_px"] // 2, int(self._parameter["snapshot_global_inset_px"])),
        )
        self._parameter["snapshot_timeout_ms"] = max(100, int(self._parameter["snapshot_timeout_ms"]))
        self._parameter["tf_timeout_s"] = max(0.05, float(self._parameter["tf_timeout_s"]))
        self._parameter["local_costmap_max_age_s"] = max(0.1, float(self._parameter["local_costmap_max_age_s"]))
        self._parameter["dynamic_layer_max_age_s"] = max(0.1, float(self._parameter["dynamic_layer_max_age_s"]))
        self._parameter["startup_grace_s"] = max(0.0, float(self._parameter["startup_grace_s"]))
        self._lock = threading.Lock()
        self._cache: Dict[str, Cached] = {}
        # Rendering may take longer than a costmap publication period.  Keep
        # subscriptions and the service in different groups so a request can
        # never starve freshness-critical cache updates.
        self._cache_callbacks = MutuallyExclusiveCallbackGroup()
        self._service_callbacks = MutuallyExclusiveCallbackGroup()
        self._tf = Buffer(cache_time=Duration(seconds=10.0))
        self._listener = TransformListener(self._tf, self)
        grid_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscribe = lambda message_type, topic, callback, qos: self.create_subscription(
            message_type, topic, callback, qos, callback_group=self._cache_callbacks)
        subscribe(
            OccupancyGrid, self._parameter["local_costmap_topic"],
            lambda msg: self._cache_message("local", msg), grid_qos)
        subscribe(
            OccupancyGrid, self._parameter["global_costmap_topic"],
            lambda msg: self._cache_message("global", msg), grid_qos)
        subscribe(
            OccupancyGridUpdate, self._parameter["local_costmap_updates_topic"],
            lambda msg: self._cache_grid_update("local", msg), 10)
        subscribe(
            OccupancyGridUpdate, self._parameter["global_costmap_updates_topic"],
            lambda msg: self._cache_grid_update("global", msg), 10)
        projected_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                   reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        subscribe(
            ProjectedKeepoutState, self._parameter["projected_keepouts_topic"],
            lambda msg: self._cache_message("keepout", msg), projected_qos)
        subscribe(
            PolygonStamped, self._parameter["local_footprint_topic"],
            lambda msg: self._cache_message("footprint", msg), 10)
        subscribe(
            PolygonStamped, self._parameter["stop_zone_topic"],
            lambda msg: self._cache_message("stop_zone", msg), 10)
        subscribe(
            MarkerArray, self._parameter["collision_polygons_topic"],
            lambda msg: self._cache_message("collision", msg), 10)
        subscribe(
            LaserScan, self._parameter["scan_topic"],
            lambda msg: self._cache_message("scan", msg), qos_profile_sensor_data)
        subscribe(Path, self._parameter["plan_topic"], lambda msg: self._cache_message("plan", msg), 10)
        self.create_service(
            GetNavSnapshot,
            self._parameter["get_snapshot_service"],
            self._on_snapshot,
            callback_group=self._service_callbacks,
        )

    def _cache_message(self, key: str, message: Any) -> None:
        with self._lock:
            previous = self._cache.get(key)
            first = previous.first_received_monotonic if previous else time.monotonic()
            self._cache[key] = Cached(message, first)

    def _cache_grid_update(self, key: str, update: OccupancyGridUpdate) -> None:
        with self._lock:
            previous = self._cache.get(key)
            if previous is None:
                return
            updated = apply_grid_update(previous.message, update)
            if updated is not None:
                self._cache[key] = Cached(updated, previous.first_received_monotonic)

    def _copy_cache(self) -> Dict[str, Cached]:
        with self._lock:
            return dict(self._cache)

    def _stamp_age_ok(self, cached: Cached, required: bool) -> bool:
        stamp = self._message_stamp(cached.message)
        stamp_ns = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(getattr(stamp, "nanosec", 0))
        now_ns = self.get_clock().now().nanoseconds
        max_age = float(self._parameter["local_costmap_max_age_s"] if required else self._parameter["dynamic_layer_max_age_s"])
        return is_fresh(
            stamp_ns, now_ns, cached.first_received_monotonic, time.monotonic(),
            max_age, float(self._parameter["startup_grace_s"]),
        )

    @staticmethod
    def _message_stamp(message: Any) -> TimeMsg:
        header = getattr(message, "header", None)
        if header is not None:
            return header.stamp
        newest = TimeMsg()
        for marker in getattr(message, "markers", ()):
            stamp = marker.header.stamp
            if (stamp.sec, stamp.nanosec) > (newest.sec, newest.nanosec):
                newest = stamp
        return newest

    @staticmethod
    def _grid(message: OccupancyGrid, transform: Optional[Transform2D] = None) -> Optional[Grid]:
        info = message.info
        values = (info.resolution, info.origin.position.x, info.origin.position.y)
        if (info.width <= 0 or info.height <= 0 or not all(math.isfinite(value) for value in values)
                or info.resolution <= 0.0 or len(message.data) != info.width * info.height):
            return None
        return Grid(message.header.frame_id, float(info.resolution),
                    (float(info.origin.position.x), float(info.origin.position.y)),
                    int(info.width), int(info.height), message.data, transform)

    def _transform(self, target: str, source: str, stamp: TimeMsg | Time) -> Optional[Transform2D]:
        if not target or not source or target == source:
            return Transform2D(source, target, 0.0, 0.0, 0.0)
        try:
            lookup_time = stamp if isinstance(stamp, Time) else Time.from_msg(stamp)
            transform = self._tf.lookup_transform(target, source, lookup_time, Duration(seconds=float(self._parameter["tf_timeout_s"])))
        except TransformException:
            return None
        q = transform.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        t = transform.transform.translation
        return Transform2D(source, target, float(t.x), float(t.y), yaw)

    def _polyline(self, message: PolygonStamped, color: Tuple[int, int, int], target: str) -> Optional[Polyline]:
        points = tuple((float(point.x), float(point.y)) for point in message.polygon.points)
        source = message.header.frame_id or str(self._parameter["base_frame"])
        transform = self._transform(target, source, message.header.stamp)
        return Polyline(source, points, color, 2, True, transform) if len(points) >= 3 and transform else None

    def _scan(self, message: LaserScan, target: str) -> Optional[Polyline]:
        points = []
        for index, distance in enumerate(message.ranges):
            if not math.isfinite(distance) or distance < message.range_min or distance > message.range_max:
                continue
            angle = message.angle_min + index * message.angle_increment
            points.append((float(distance * math.cos(angle)), float(distance * math.sin(angle))))
        source = message.header.frame_id or str(self._parameter["base_frame"])
        transform = self._transform(target, source, message.header.stamp)
        return Polyline(source, tuple(points), (0, 80, 255), 1, False, transform) if points and transform else None

    def _path(self, message: Path, target: str, color: Tuple[int, int, int]) -> Optional[Polyline]:
        points = tuple((float(pose.pose.position.x), float(pose.pose.position.y)) for pose in message.poses)
        source = message.header.frame_id or (message.poses[0].header.frame_id if message.poses else "") or str(self._parameter["base_frame"])
        transform = self._transform(target, source, message.header.stamp)
        return Polyline(source, points, color, 2, False, transform) if len(points) >= 2 and transform else None

    def _collision(self, message: MarkerArray, target: str) -> Tuple[Polyline, ...]:
        result = []
        for marker in message.markers:
            if len(marker.points) < 2:
                continue
            quaternion = marker.pose.orientation
            yaw = math.atan2(
                2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
                1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
            )
            cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
            points = tuple((
                float(marker.pose.position.x + cos_yaw * point.x - sin_yaw * point.y),
                float(marker.pose.position.y + sin_yaw * point.x + cos_yaw * point.y),
            ) for point in marker.points)
            color = (int(marker.color.b * 255), int(marker.color.g * 255), int(marker.color.r * 255))
            source = marker.header.frame_id or str(self._parameter["base_frame"])
            transform = self._transform(target, source, marker.header.stamp)
            if transform is None:
                continue
            marker_color = color if any(color) else (0, 200, 255)
            if marker.type == Marker.LINE_LIST:
                result.extend(
                    Polyline(source, tuple(points[index:index + 2]), marker_color, 2, False, transform)
                    for index in range(0, len(points) - 1, 2)
                )
            else:
                closed = marker.type == Marker.LINE_STRIP and len(points) >= 3 and points[0] != points[-1]
                result.append(Polyline(source, points, marker_color, 2, closed, transform))
        return tuple(result)

    def _keepouts(self, message: ProjectedKeepoutState, target: str) -> Tuple[KeepoutPolygon, ...]:
        source = message.header.frame_id or "map"
        # Keepouts are fixed in map.  Their publication stamp identifies the
        # accepted geometry revision, not a historical map->odom pose.  Use
        # the current transform so an old transient-local state still follows
        # the latest localization correction.
        transform = self._transform(target, source, Time())
        if transform is None:
            return ()
        return tuple(
            KeepoutPolygon(
                source,
                tuple((float(point.x), float(point.y)) for point in polygon.outer.points),
                tuple(tuple((float(point.x), float(point.y)) for point in hole.points) for hole in polygon.holes),
                transform,
            )
            for polygon in message.polygons
            if len(polygon.outer.points) >= 3
        )

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
        dynamic = lambda key: cache.get(key) if key in cache and self._stamp_age_ok(cache[key], False) else None
        global_cached, keepout_cached = dynamic("global"), cache.get("keepout")
        global_grid = self._grid(global_cached.message) if global_cached else None
        keepouts, global_keepouts = (), ()
        if keepout_cached is not None:
            keepouts = self._keepouts(keepout_cached.message, local.frame_id)
            if global_grid is not None:
                global_keepouts = self._keepouts(keepout_cached.message, global_grid.frame_id)
        footprint_cached, stop_cached = dynamic("footprint"), dynamic("stop_zone")
        scan_cached, plan_cached, collision_cached = dynamic("scan"), dynamic("plan"), dynamic("collision")
        global_plan = self._path(plan_cached.message, global_grid.frame_id, (96, 255, 96)) if plan_cached and global_grid else None
        robot_global_transform = self._transform(global_grid.frame_id, base, local_cached.message.header.stamp) if global_grid else None
        scene = SnapshotScene(
            local_costmap=local,
            center_xy=(robot.x, robot.y),
            extent_m=self._parameter["snapshot_extent_m"],
            size_px=self._parameter["snapshot_size_px"],
            global_inset_px=self._parameter["snapshot_global_inset_px"],
            vector_keepouts=keepouts,
            global_vector_keepouts=global_keepouts,
            global_costmap=global_grid,
            footprint=self._polyline(footprint_cached.message, (0, 255, 0), local.frame_id) if footprint_cached else None,
            stop_zone=self._polyline(stop_cached.message, (0, 0, 255), local.frame_id) if stop_cached else None,
            collision_polygons=self._collision(collision_cached.message, local.frame_id) if collision_cached else (),
            scan=self._scan(scan_cached.message, local.frame_id) if scan_cached else None,
            plan=self._path(plan_cached.message, local.frame_id, (64, 255, 64)) if plan_cached else None,
            global_plan=global_plan,
            robot_global=(robot_global_transform.x, robot_global_transform.y) if robot_global_transform else None,
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
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
