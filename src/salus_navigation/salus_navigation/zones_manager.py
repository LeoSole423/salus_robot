"""Own dynamic GeoJSON no-go zones and publish accepted vector state atomically."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Point32, Polygon
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_localization.srv import FromLL
from std_srvs.srv import Trigger

from salus_interfaces.msg import ProjectedKeepoutPolygon, ProjectedKeepoutState
from salus_interfaces.srv import GetZonesState, SetZonesGeoJson
from .zones_geojson import (
    feature_and_polygon_counts, iter_polygons, normalize_geojson,
)


EMPTY_GEOJSON = {"type": "FeatureCollection", "features": []}


def zones_document_is_empty(document: dict[str, Any]) -> bool:
    """Return whether a normalized persisted document contains no zones."""
    return document.get("type") == "FeatureCollection" and not document.get("features")


def _polygon_message(points: list[dict[str, float]]) -> Polygon:
    message = Polygon()
    for point in points:
        vertex = Point32()
        vertex.x = float(point["x"])
        vertex.y = float(point["y"])
        vertex.z = 0.0
        message.points.append(vertex)
    return message


def projected_keepout_state_message(
    polygons: list[dict[str, Any]],
    *,
    frame_id: str,
    revision: int,
    stamp: Any,
) -> ProjectedKeepoutState:
    """Build the accepted map-frame vector keepout state for publication."""
    message = ProjectedKeepoutState()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.revision = int(revision)
    for polygon in polygons:
        if not polygon["enabled"]:
            continue
        item = ProjectedKeepoutPolygon()
        item.zone_id = str(polygon["id"])
        item.outer = _polygon_message(polygon["outer_xy"])
        item.holes = [_polygon_message(hole) for hole in polygon["holes_xy"]]
        message.polygons.append(item)
    return message


class ZonesManager(Node):
    """GeoJSON API boundary; only projected, persisted candidates become live."""

    def __init__(self) -> None:
        super().__init__("zones_manager")
        defaults = {
            "runtime_dir": "runtime/zones", "map_frame": "map",
            "fromll_service": "/fromLL", "fromll_service_fallback": "/navsat_transform/fromLL",
            "use_keepout": True,
            "service_timeout_s": 4.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.runtime_dir = Path(str(value("runtime_dir"))).resolve()
        self.geojson_path = self.runtime_dir / "no_go_zones.geojson"
        self.map_frame = str(value("map_frame"))
        self.use_keepout = bool(value("use_keepout"))
        self.timeout_s = max(0.2, float(value("service_timeout_s")))
        self._lock = threading.Lock()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()
        self._document = EMPTY_GEOJSON
        self._document_text = json.dumps(EMPTY_GEOJSON, separators=(",", ":"))
        self._mask_ready, self._mask_source = False, "none"
        self._projected_revision = 0
        projected_qos = QoSProfile(depth=1)
        projected_qos.reliability = ReliabilityPolicy.RELIABLE
        projected_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._projected_keepouts_pub = self.create_publisher(
            ProjectedKeepoutState,
            "/zones_manager/projected_keepouts",
            projected_qos,
        )
        self._fromll_clients = [
            self.create_client(FromLL, str(value("fromll_service")), callback_group=self._client_group),
            self.create_client(FromLL, str(value("fromll_service_fallback")), callback_group=self._client_group),
        ]
        self.create_service(SetZonesGeoJson, "/zones_manager/set_geojson", self._set_geojson, callback_group=self._service_group)
        self.create_service(GetZonesState, "/zones_manager/get_state", self._get_state, callback_group=self._service_group)
        self.create_service(Trigger, "/zones_manager/reload_from_disk", self._reload, callback_group=self._service_group)
        self._initialization_timer = self.create_timer(
            0.5, self._load_initial_state, callback_group=self._service_group
        )

    def _load_initial_state(self) -> None:
        self._initialization_timer.cancel()
        try:
            text = self.geojson_path.read_text(encoding="utf-8")
            document = normalize_geojson(json.loads(text))
        except FileNotFoundError:
            document = EMPTY_GEOJSON
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignoring invalid persisted zones: {exc}")
            document = EMPTY_GEOJSON

        ok, error, _, _ = self._apply(
            document, persist=not self.geojson_path.exists()
        )
        if not ok:
            self.get_logger().error(f"initial projected zones unavailable: {error}")

    def _await(self, client, request: Any):
        if not client.wait_for_service(timeout_sec=self.timeout_s):
            return None, "service unavailable"
        future = client.call_async(request)
        deadline = time.monotonic() + self.timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return None, "service timeout"
        try:
            return future.result(), ""
        except Exception as exc:  # ROS middleware errors need a service response.
            return None, str(exc)

    def _project(self, document: dict[str, Any]):
        client = next((item for item in self._fromll_clients if item.service_is_ready()), None)
        polygons: list[dict[str, Any]] = []
        for polygon in iter_polygons(document):
            converted = {"id": polygon["id"], "enabled": polygon["enabled"], "outer_xy": [], "holes_xy": []}
            for key, rings in (("outer_xy", [polygon["outer_ll"]]), ("holes_xy", polygon["holes_ll"])):
                for ring in rings:
                    output = []
                    for lon, lat in ring:
                        if client is None:
                            return None, "fromLL service unavailable"
                        request = FromLL.Request(); request.ll_point.latitude = lat; request.ll_point.longitude = lon
                        response, error = self._await(client, request)
                        if response is None:
                            fallback = self._fromll_clients[1] if client is self._fromll_clients[0] else None
                            if fallback is not None and fallback.service_is_ready():
                                client = fallback; response, error = self._await(client, request)
                        if response is None:
                            return None, f"fromLL conversion failed: {error}"
                        output.append({"x": float(response.map_point.x), "y": float(response.map_point.y)})
                    if key == "outer_xy": converted[key] = output
                    else: converted[key].append(output)
            polygons.append(converted)
        return polygons, ""

    def _persist_document(self, document: dict[str, Any]) -> tuple[bool, str]:
        """Atomically replace the GeoJSON only after a vector candidate exists."""
        try:
            self.runtime_dir.mkdir(parents=True, exist_ok=True)
            staged = self.runtime_dir / ".no_go_zones.tmp.geojson"
            staged.write_text(
                json.dumps(document, separators=(",", ":")) + "\n",
                encoding="utf-8",
            )
            staged.replace(self.geojson_path)
        except OSError as exc:
            return False, f"failed to persist zones: {exc}"
        return True, ""

    def _publish_projected_state(
        self, polygons: list[dict[str, Any]], revision: int
    ) -> None:
        message = projected_keepout_state_message(
            polygons,
            frame_id=self.map_frame,
            revision=revision,
            stamp=self.get_clock().now().to_msg(),
        )
        self._projected_keepouts_pub.publish(message)

    def _activate_document(
        self,
        document: dict[str, Any],
        projected_polygons: list[dict[str, Any]],
        *,
        mask_source: str,
    ) -> None:
        document_text = json.dumps(document, separators=(",", ":"))
        with self._lock:
            self._document = document
            self._document_text = document_text
            self._mask_ready = True
            self._mask_source = mask_source
            self._projected_revision += 1
            revision = self._projected_revision
        self._publish_projected_state(projected_polygons, revision)

    def _apply(self, document: dict[str, Any], *, persist: bool) -> tuple[bool, str, int, int]:
        if not persist:
            document = normalize_geojson(document)
        candidate_document = document if self.use_keepout else EMPTY_GEOJSON
        projected_polygons, error = self._project(candidate_document)
        if projected_polygons is None:
            return False, error, 0, 0
        if persist:
            ok, error = self._persist_document(document)
            if not ok:
                return False, error, 0, 0
        features, polygons = feature_and_polygon_counts(document)
        # This is the commit point: candidate projection and (when requested)
        # persistence succeeded.  The transient-local publication follows the
        # authoritative in-process update and an error never reports success.
        self._activate_document(
            document,
            projected_polygons,
            mask_source="projected_vector_state",
        )
        return True, "", features, polygons

    def _set_geojson(self, request, response):
        try:
            document = normalize_geojson(json.loads(request.geojson))
        except (ValueError, json.JSONDecodeError) as exc:
            response.ok, response.error, response.map_reloaded = False, str(exc), False
            return response
        ok, error, features, polygons = self._apply(document, persist=True)
        response.ok, response.error, response.map_reloaded = ok, error, ok
        response.feature_count, response.polygon_count = features, polygons
        return response

    def _get_state(self, _request, response):
        with self._lock:
            response.ok, response.error, response.frame_id = True, "", self.map_frame
            response.mask_ready, response.mask_source, response.geojson = self._mask_ready, self._mask_source, self._document_text
        return response

    def _reload(self, _request, response):
        try:
            document = normalize_geojson(json.loads(self.geojson_path.read_text(encoding="utf-8")))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            response.success, response.message = False, str(exc)
            return response
        ok, error, features, polygons = self._apply(document, persist=False)
        response.success = ok
        response.message = error if not ok else f"reloaded (features={features}, polygons={polygons})"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZonesManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown(); node.destroy_node(); rclpy.shutdown()
