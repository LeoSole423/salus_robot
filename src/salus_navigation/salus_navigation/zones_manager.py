"""Own dynamic GeoJSON no-go zones and reload the Nav2 keepout mask atomically."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path
from typing import Any

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ClearEntireCostmap, LoadMap
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_localization.srv import FromLL
from std_srvs.srv import Trigger

from salus_interfaces.srv import GetZonesState, SetZonesGeoJson
from .zones_geojson import (
    cost_mask_from_binary, feature_and_polygon_counts, iter_polygons,
    normalize_geojson, rasterize_polygons, scale_image_from_costs,
)


EMPTY_GEOJSON = {"type": "FeatureCollection", "features": []}


class ZonesManager(Node):
    """GeoJSON API boundary; no partial mask replaces a previously active one."""

    def __init__(self) -> None:
        super().__init__("zones_manager")
        defaults = {
            "runtime_dir": "runtime/zones", "map_frame": "map",
            "fromll_service": "/fromLL", "fromll_service_fallback": "/navsat_transform/fromLL",
            "load_map_service": "/keepout_filter_mask_server/load_map",
            "load_map_state_service": "/keepout_filter_mask_server/get_state",
            "clear_global_costmap_service": "/global_costmap/clear_entirely_global_costmap",
            "mask_origin_x": -150.0, "mask_origin_y": -150.0,
            "mask_width": 3000, "mask_height": 3000, "mask_resolution": 0.1,
            "buffer_margin_m": 0.0, "degrade_enabled": True, "degrade_radius_m": 1.5,
            "degrade_edge_cost": 12, "degrade_min_cost": 1,
            "use_keepout": True,
            "service_timeout_s": 4.0,
            "initial_reload_retry_s": 1.0,
            "initial_reload_max_attempts": 20,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.runtime_dir = Path(str(value("runtime_dir"))).resolve()
        self.geojson_path = self.runtime_dir / "no_go_zones.geojson"
        self.mask_path = self.runtime_dir / "keepout_mask.pgm"
        self.mask_yaml_path = self.runtime_dir / "keepout_mask.yaml"
        self.map_frame = str(value("map_frame"))
        self.origin_x, self.origin_y = float(value("mask_origin_x")), float(value("mask_origin_y"))
        self.width, self.height, self.resolution = int(value("mask_width")), int(value("mask_height")), float(value("mask_resolution"))
        self.buffer_margin_m = max(0.0, float(value("buffer_margin_m")))
        self.degrade_enabled = bool(value("degrade_enabled"))
        self.use_keepout = bool(value("use_keepout"))
        self.degrade_radius_m = max(0.0, float(value("degrade_radius_m")))
        self.degrade_edge_cost = min(99, max(1, int(value("degrade_edge_cost"))))
        self.degrade_min_cost = min(self.degrade_edge_cost, max(1, int(value("degrade_min_cost"))))
        self.timeout_s = max(0.2, float(value("service_timeout_s")))
        self._initial_reload_retry_s = max(0.1, float(value("initial_reload_retry_s")))
        self._initial_reload_max_attempts = max(1, int(value("initial_reload_max_attempts")))
        self._initial_reload_attempt = 0
        self._lock = threading.Lock()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()
        self._document = EMPTY_GEOJSON
        self._document_text = json.dumps(EMPTY_GEOJSON, separators=(",", ":"))
        self._mask_ready, self._mask_source = False, "none"
        self._fromll_clients = [
            self.create_client(FromLL, str(value("fromll_service")), callback_group=self._client_group),
            self.create_client(FromLL, str(value("fromll_service_fallback")), callback_group=self._client_group),
        ]
        self._load_map = self.create_client(
            LoadMap, str(value("load_map_service")), callback_group=self._client_group
        )
        self._load_map_state = self.create_client(
            GetState,
            str(value("load_map_state_service")),
            callback_group=self._client_group,
        )
        self._clear_global = self.create_client(ClearEntireCostmap, str(value("clear_global_costmap_service")), callback_group=self._client_group)
        self.create_service(SetZonesGeoJson, "/zones_manager/set_geojson", self._set_geojson, callback_group=self._service_group)
        self.create_service(GetZonesState, "/zones_manager/get_state", self._get_state, callback_group=self._service_group)
        self.create_service(Trigger, "/zones_manager/reload_from_disk", self._reload, callback_group=self._service_group)
        # Defer and retry the initial reload: the map server is lifecycle
        # managed independently and may not be active on its first attempt.
        # Initialization and operator updates both replace the same persisted
        # mask and synchronously wait on map-server clients.  Serialize them so
        # they cannot occupy every executor thread while their service
        # responses are waiting to run in the reentrant client group.
        self._initialization_timer = self.create_timer(
            0.5, self._load_initial_state, callback_group=self._service_group
        )

    def _schedule_initial_reload(self) -> None:
        self._initialization_timer = self.create_timer(
            self._initial_reload_retry_s,
            self._load_initial_state,
            callback_group=self._service_group,
        )

    def _load_initial_state(self) -> None:
        # Treat this repeating ROS timer as a one-shot.  Mask generation may
        # take longer than its period; cancelling up front prevents another
        # initialization callback from already being queued behind this one.
        self._initialization_timer.cancel()

        # A lifecycle service being discoverable does not mean LoadMap is
        # legal.  Wait for the map server's actual ACTIVE state before doing
        # the expensive 3000x3000 mask generation or sending LoadMap.
        active, error = self._require_map_server_active()
        if not active:
            self.get_logger().info(
                f"initial zone mask waiting for active map server: {error}"
            )
            self._schedule_initial_reload()
            return

        self._initial_reload_attempt += 1
        try:
            text = self.geojson_path.read_text(encoding="utf-8")
            document = normalize_geojson(json.loads(text))
        except FileNotFoundError:
            document = EMPTY_GEOJSON
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f"ignoring invalid persisted zones: {exc}")
            document = EMPTY_GEOJSON
        ok, error, _, _ = self._apply(document, persist=not self.geojson_path.exists())
        if ok:
            return
        if self._initial_reload_attempt >= self._initial_reload_max_attempts:
            self.get_logger().error(f"initial zone mask unavailable: {error}")
            return
        self.get_logger().info(
            "initial zone mask not ready; retrying "
            f"({self._initial_reload_attempt}/{self._initial_reload_max_attempts}): {error}"
        )
        self._schedule_initial_reload()

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

    def _map_server_state(self) -> tuple[str | None, str]:
        response, error = self._await(self._load_map_state, GetState.Request())
        if response is None:
            return None, f"keepout map lifecycle unavailable: {error}"
        return str(response.current_state.label), ""

    def _require_map_server_active(self) -> tuple[bool, str]:
        state, error = self._map_server_state()
        if state is None:
            return False, error
        if state != "active":
            return False, f"keepout map server not active: {state}"
        return True, ""

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

    def _write_mask(self, document: dict[str, Any]) -> tuple[bool, str]:
        if not self.use_keepout:
            document = EMPTY_GEOJSON
        polygons, error = self._project(document)
        if polygons is None:
            return False, error
        image, _, _ = rasterize_polygons(polygons, self.width, self.height, self.resolution, self.origin_x, self.origin_y, self.buffer_margin_m)
        costs = cost_mask_from_binary(image, self.resolution, self.degrade_radius_m if self.degrade_enabled else 0.0, self.degrade_edge_cost, self.degrade_min_cost)
        scale = scale_image_from_costs(costs)
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        staged_image = self.runtime_dir / ".keepout_mask.tmp.pgm"
        staged_yaml = self.runtime_dir / ".keepout_mask.tmp.yaml"
        staged_geojson = self.runtime_dir / ".no_go_zones.tmp.geojson"
        if not cv2.imwrite(str(staged_image), scale):
            return False, "failed to write keepout mask"
        staged_yaml.write_text(json.dumps({"image": self.mask_path.name, "mode": "scale", "resolution": self.resolution, "origin": [self.origin_x, self.origin_y, 0.0], "negate": 0, "occupied_thresh": 1.0, "free_thresh": 0.0}, indent=2) + "\n", encoding="utf-8")
        staged_geojson.write_text(json.dumps(document, separators=(",", ":")) + "\n", encoding="utf-8")
        staged_image.replace(self.mask_path); staged_yaml.replace(self.mask_yaml_path); staged_geojson.replace(self.geojson_path)
        return True, ""

    def _reload_map(self) -> tuple[bool, str]:
        request = LoadMap.Request(); request.map_url = str(self.mask_yaml_path)
        response, error = self._await(self._load_map, request)
        if response is None or int(response.result) != LoadMap.Response.RESULT_SUCCESS:
            return False, f"load_map failed: {error or int(response.result)}"
        response, error = self._await(self._clear_global, ClearEntireCostmap.Request())
        if response is None:
            # The map is already active. Clearing is a replanning aid, not a
            # reason to claim that a successfully loaded mask was rejected.
            self.get_logger().warning(f"global costmap clear failed after map load: {error}")
        return True, ""

    def _apply(self, document: dict[str, Any], *, persist: bool) -> tuple[bool, str, int, int]:
        # Do not generate/stage a replacement mask until LoadMap is causally
        # legal for the lifecycle-managed map server.
        active, error = self._require_map_server_active()
        if not active:
            return False, error, 0, 0
        # Stage files first.  Only publish them as current after Nav2 accepts the new map.
        old_document, old_text = self._document, self._document_text
        if not persist:
            document = normalize_geojson(document)
        ok, error = self._write_mask(document)
        if not ok:
            return False, error, 0, 0
        ok, error = self._reload_map()
        if not ok:
            # Restore the known-good mask for the next successful reload; current Nav2 map is unchanged.
            self._write_mask(old_document)
            return False, error, 0, 0
        features, polygons = feature_and_polygon_counts(document)
        with self._lock:
            self._document, self._document_text = document, json.dumps(document, separators=(",", ":"))
            self._mask_ready, self._mask_source = True, "map_server_load_map+global_costmap_clear"
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
