"""ROS-only adapter for the Cockpit protocol.

The node owns ROS clients, publishers and subscriptions. It never opens a
socket and delegates transport correlation and operator ownership elsewhere.
"""

from __future__ import annotations

import asyncio
import base64
from copy import deepcopy
import json
import math
from pathlib import Path
from threading import Lock
import time
from typing import Any, Callable, Iterable, Mapping

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from nav_msgs.msg import Odometry
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import BatteryState, LaserScan, NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger

from salus_interfaces.msg import (
    BatteryMissionGuard,
    CapabilityState,
    CmdVelFinal,
    DriveTelemetry,
    GnssRtkStatus,
    NavEvent,
    NavTelemetry,
    SystemCapabilities,
)
from salus_interfaces.srv import (
    BrakeNav,
    CancelNavGoal,
    CancelPatrolMission,
    CancelRouteMission,
    GetNavSnapshot,
    GetNavState,
    GetPatrolMissionState,
    GetRouteMissionState,
    GetZonesState,
    RequestReturnHome,
    SetManualMode,
    SetNavGoalLL,
    SetNavigationProfile,
    SetPatrolMissionLL,
    SetRouteMissionLL,
    SetZonesGeoJson,
    CameraPan,
    CameraStatus,
    CameraPtz,
    CameraPreset,
    CameraSavePreset,
    CameraPtzState,
)

from .compact_telemetry import (
    CompactTelemetryPolicy,
    normalize_telemetry_profile,
    positive_rate,
)
from .protocol import OperatorRequest, ack
from .waypoint_repository import AtomicWaypointRepository, normalize_document


class RosGatewayError(RuntimeError):
    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code


def capability_state_label(value: object) -> str:
    """Project typed ROS constants into a stable operator-facing label."""

    labels = {
        CapabilityState.STATE_UNKNOWN: "unknown",
        CapabilityState.STATE_NOT_INSTALLED: "not_installed",
        CapabilityState.STATE_DISABLED_BY_PROFILE: "disabled_by_profile",
        CapabilityState.STATE_UNAVAILABLE: "unavailable",
        CapabilityState.STATE_INVALID: "invalid",
        CapabilityState.STATE_STALE: "stale",
        CapabilityState.STATE_FAILED: "failed",
        CapabilityState.STATE_ENABLED_BY_PROFILE: "enabled_by_profile",
        CapabilityState.STATE_READY: "ready",
    }
    try:
        return labels.get(int(value), "unknown")
    except (TypeError, ValueError):
        return "unknown"


# These are deliberately tables rather than a derived enum-name conversion:
# Cockpit consumes stable, human-readable values even if ROS constant names
# evolve. Unknown future values must remain explicit rather than looking valid.
GNSS_FIX_QUALITY_LABELS = {
    GnssRtkStatus.UNKNOWN: "unknown",
    GnssRtkStatus.NO_FIX: "no_fix",
    GnssRtkStatus.AUTONOMOUS: "autonomous",
    GnssRtkStatus.DGPS: "dgps",
    GnssRtkStatus.RTK_FLOAT: "rtk_float",
    GnssRtkStatus.RTK_FIXED: "rtk_fixed",
}
GNSS_ACQUISITION_STATE_LABELS = {
    GnssRtkStatus.ACQUISITION_DISABLED: "disabled",
    GnssRtkStatus.ACQUISITION_DISCONNECTED: "disconnected",
    GnssRtkStatus.ACQUISITION_CONNECTED_NO_DATA: "connected_no_data",
    GnssRtkStatus.ACQUISITION_RECEIVING: "receiving",
    GnssRtkStatus.ACQUISITION_STALE: "stale",
    GnssRtkStatus.ACQUISITION_ERROR: "error",
}
GNSS_DELIVERY_BACKEND_LABELS = {
    GnssRtkStatus.BACKEND_DISABLED: "disabled",
    GnssRtkStatus.BACKEND_PIXHAWK_MAVROS: "pixhawk_mavros",
    GnssRtkStatus.BACKEND_DIRECT_USB: "direct_usb",
}
GNSS_DELIVERY_STATE_LABELS = {
    GnssRtkStatus.DELIVERY_DISABLED: "disabled",
    GnssRtkStatus.DELIVERY_IDLE: "idle",
    GnssRtkStatus.DELIVERY_DELIVERING: "delivering",
    GnssRtkStatus.DELIVERY_STALE: "stale",
    GnssRtkStatus.DELIVERY_ERROR: "error",
}


def _enum_label(labels: Mapping[int, str], value: object) -> str:
    try:
        return labels.get(int(value), "unknown")
    except (TypeError, ValueError):
        return "unknown"


def gnss_rtk_status_payload(message: GnssRtkStatus) -> dict[str, Any]:
    """Project the canonical RTK observation without inferring GNSS quality.

    The incoming contract owns the distinction between correction freshness and
    receiver fix quality. This bridge only translates it to JSON-safe operator
    fields; in particular it never derives an RTK solution from RTCM counters.
    """
    satellites_visible = int(message.satellites_visible)
    correction_age_s = _finite_or_none(message.correction_age_s)
    if correction_age_s is not None and correction_age_s < 0.0:
        correction_age_s = None
    return {
        "available": True,
        "source": "gnss_rtk_status",
        "fix_quality": _enum_label(GNSS_FIX_QUALITY_LABELS, message.fix_quality),
        "acquisition_state": _enum_label(
            GNSS_ACQUISITION_STATE_LABELS, message.acquisition_state
        ),
        "delivery_backend": _enum_label(
            GNSS_DELIVERY_BACKEND_LABELS, message.delivery_backend
        ),
        "delivery_state": _enum_label(
            GNSS_DELIVERY_STATE_LABELS, message.delivery_state
        ),
        "receiver_fix_type": int(message.receiver_fix_type),
        "satellites_visible": None if satellites_visible == 255 else satellites_visible,
        "corrections_fresh": bool(message.corrections_fresh),
        "correction_age_s": correction_age_s,
        "received_count": int(message.received_count),
        "crc_error_count": int(message.crc_error_count),
        "source_id": str(message.source_id),
        "status_detail": str(message.status_detail),
    }


def accepts_legacy_rtk_status(typed_status_received: bool) -> bool:
    """Whether a legacy text update may replace the cached RTK projection."""
    return not typed_status_received


class CockpitRosGateway(Node):
    def __init__(self) -> None:
        super().__init__("salus_web_gateway")
        self.declare_parameter("waypoints_file", "runtime/web/waypoints.yaml")
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("service_discovery_timeout_s", 5.0)
        self.declare_parameter("long_service_timeout_s", 20.0)
        self.declare_parameter("required_service_startup_timeout_s", 20.0)
        self.declare_parameter("require_camera_service", False)
        self.declare_parameter("ws_host", "0.0.0.0")
        self.declare_parameter("ws_port", 8766)
        self.declare_parameter("enable_control_lock", True)
        self.declare_parameter("control_lock_start_locked", True)
        self.declare_parameter("control_lock_heartbeat_timeout_s", 2.5)
        self.declare_parameter("client_queue_capacity", 64)
        self.declare_parameter("telemetry_profile", "compact")
        self.declare_parameter("compact_telemetry_hz", 2.0)
        self.declare_parameter("heading_odometry_topic", "/odometry/local")
        self.declare_parameter("scan_preview_topic", "/scan_preview")
        self.declare_parameter("scan_preview_enabled", True)
        self._service_timeout_s = max(
            0.1, float(self.get_parameter("service_timeout_s").value)
        )
        self._service_discovery_timeout_s = max(
            0.0, float(self.get_parameter("service_discovery_timeout_s").value)
        )
        self._long_service_timeout_s = max(
            self._service_timeout_s,
            float(self.get_parameter("long_service_timeout_s").value),
        )
        self._required_service_startup_timeout_s = max(
            self._service_timeout_s,
            float(self.get_parameter("required_service_startup_timeout_s").value),
        )
        self._waypoints = AtomicWaypointRepository(
            Path(str(self.get_parameter("waypoints_file").value))
        )
        self._telemetry_profile = normalize_telemetry_profile(
            self.get_parameter("telemetry_profile").value
        )
        self._compact_policy = CompactTelemetryPolicy(
            max_hz=positive_rate(
                self.get_parameter("compact_telemetry_hz").value,
                "compact_telemetry_hz",
            ),
            clock=time.monotonic,
        )
        self._lock = Lock()
        self._cache: dict[str, Any] = {"connected": True, "mode": "connected"}
        # Once the canonical status has arrived it remains authoritative for
        # this process lifetime; the legacy text topic is only a migration
        # fallback and must never roll the UI back to an inferred status.
        self._typed_rtk_status_received = False
        self._broadcast: Callable[[dict[str, Any]], None] | None = None

        self._teleop = self.create_publisher(CmdVelFinal, "/cmd_vel_teleop", 10)
        self.create_subscription(
            NavTelemetry,
            "/nav_command_server/telemetry",
            self._on_nav_telemetry,
            10,
        )
        self.create_subscription(
            NavEvent, "/nav_command_server/events", self._on_nav_event, 50
        )
        self.create_subscription(
            DriveTelemetry,
            "/controller/drive_telemetry",
            self._on_drive_telemetry,
            10,
        )
        self.create_subscription(
            BatteryMissionGuard,
            "/battery_mission_guard",
            self._on_battery_guard,
            10,
        )
        self.create_subscription(
            BatteryState, "/battery_state", self._on_battery_state, 10
        )
        self.create_subscription(
            NavSatFix, "/gps/fix", self._on_gps, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("heading_odometry_topic").value),
            self._on_heading_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, "/gps/rtk_status", self._on_rtk, 10)
        self.create_subscription(
            GnssRtkStatus,
            "/salus/hardware/gnss_primary/rtk_status",
            self._on_gnss_rtk_status,
            10,
        )
        capability_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            SystemCapabilities,
            "/system/capabilities",
            self._on_capabilities,
            capability_qos,
        )
        if bool(self.get_parameter("scan_preview_enabled").value):
            self.create_subscription(
                LaserScan,
                str(self.get_parameter("scan_preview_topic").value),
                self._on_scan_preview,
                qos_profile_sensor_data,
            )
        # Use steady time: telemetry must continue to flush when Gazebo's
        # `/clock` is paused or temporarily unavailable.
        self.create_timer(
            1.0 / self._compact_policy.max_hz,
            self._on_compact_telemetry_timer,
            clock=Clock(clock_type=ClockType.STEADY_TIME),
        )

        # Do not use ``_clients``: rclpy.Node owns that attribute and its
        # executor expects it to remain a list of ROS client entities.
        self._service_clients = {
            "set_goal_ll": self.create_client(
                SetNavGoalLL, "/nav_command_server/set_goal_ll"
            ),
            "cancel_goal": self.create_client(
                CancelNavGoal, "/nav_command_server/cancel_goal"
            ),
            "brake": self.create_client(BrakeNav, "/nav_command_server/brake"),
            "set_manual_mode": self.create_client(
                SetManualMode, "/nav_command_server/set_manual_mode"
            ),
            "get_nav_state": self.create_client(
                GetNavState, "/nav_command_server/get_state"
            ),
            "set_route_ll": self.create_client(
                SetRouteMissionLL, "/route_executor/set_route_mission_ll"
            ),
            "cancel_route": self.create_client(
                CancelRouteMission, "/route_executor/cancel_route_mission"
            ),
            "get_route_state": self.create_client(
                GetRouteMissionState, "/route_executor/get_route_mission_state"
            ),
            "set_navigation_profile": self.create_client(
                SetNavigationProfile, "/route_executor/set_navigation_profile"
            ),
            "set_patrol_ll": self.create_client(
                SetPatrolMissionLL, "/route_executor/set_patrol_mission_ll"
            ),
            "cancel_patrol": self.create_client(
                CancelPatrolMission, "/route_executor/cancel_patrol_mission"
            ),
            "request_return_home": self.create_client(
                RequestReturnHome, "/route_executor/request_return_home"
            ),
            "get_patrol_state": self.create_client(
                GetPatrolMissionState, "/route_executor/get_patrol_mission_state"
            ),
            "set_zones_geojson": self.create_client(
                SetZonesGeoJson, "/zones_manager/set_geojson"
            ),
            "get_zones_state": self.create_client(
                GetZonesState, "/zones_manager/get_state"
            ),
            "load_zones_file": self.create_client(
                Trigger, "/zones_manager/reload_from_disk"
            ),
            "get_nav_snapshot": self.create_client(
                GetNavSnapshot, "/nav_snapshot_server/get_nav_snapshot"
            ),
            "camera_pan": self.create_client(CameraPan, "/camara/camera_pan"),
            "camera_zoom_toggle": self.create_client(Trigger, "/camara/camera_zoom_toggle"),
            "get_camera_status": self.create_client(CameraStatus, "/camara/camera_status"),
            "camera_ptz_move": self.create_client(CameraPtz, "/camara/camera_ptz"),
            "camera_ptz_preset": self.create_client(CameraPreset, "/camara/camera_preset"),
            "camera_ptz_set_preset": self.create_client(
                CameraSavePreset, "/camara/camera_save_preset"
            ),
            "get_camera_ptz_state": self.create_client(
                CameraPtzState, "/camara/camera_ptz_state"
            ),
        }

    def set_broadcast_callback(
        self, callback: Callable[[dict[str, Any]], None]
    ) -> None:
        self._broadcast = callback

    async def initial_state(self) -> dict[str, Any]:
        payload = self._cached_payload("state")
        calls = {
            "nav": ("get_nav_state", GetNavState.Request()),
            "route_mission": ("get_route_state", GetRouteMissionState.Request()),
            "patrol_mission": ("get_patrol_state", GetPatrolMissionState.Request()),
            "zones": ("get_zones_state", GetZonesState.Request()),
        }
        results = await asyncio.gather(
            *(self._call(name, request) for name, request in calls.values()),
            return_exceptions=True,
        )
        for section, result in zip(calls, results):
            if isinstance(result, Exception):
                payload[section] = {"available": False, "error": str(result)}
                continue
            values = _message_dict(result)
            values["available"] = bool(values.get("ok", True))
            payload[section] = values
        nav = payload.get("nav", {})
        if isinstance(nav, Mapping):
            for name in (
                "goal_active", "manual_enabled", "manual_linear_x_cmd",
                "manual_angular_z_cmd", "robot_lat", "robot_lon",
            ):
                if name in nav:
                    payload[name] = nav[name]
        payload["ok"] = True
        return payload

    async def dispatch(self, request: OperatorRequest) -> Iterable[dict[str, Any]]:
        try:
            if request.op == "get_state":
                state = await self.initial_state()
                if request.request_id is not None:
                    state["client_req_id"] = request.request_id
                return [state]
            if request.op == "save_waypoints_file":
                document = normalize_document({
                    "waypoints": request.fields.get("waypoints"),
                    "patrol_profile": request.fields.get("patrol_profile"),
                })
                self._waypoints.save(document)
                return [ack(request, ok=True, waypoint_count=len(document.waypoints))]
            if request.op == "load_waypoints_file":
                document = self._waypoints.load()
                return [ack(
                    request,
                    ok=True,
                    waypoint_count=len(document.waypoints),
                    waypoints=deepcopy(list(document.waypoints)),
                    patrol_profile=deepcopy(document.patrol_profile),
                )]
            if request.op == "set_manual_cmd":
                self._publish_manual(request.fields)
                return [ack(request, ok=True)]
            if request.op == "set_sensor_info_view":
                tab = str(request.fields.get("tab") or "general")
                implemented = tab == "general"
                response = ack(
                    request,
                    ok=True,
                    enabled=request.fields["enabled"],
                    tab=tab,
                    interval_s=request.fields.get("interval_s", 0.5),
                    topic_name=request.fields.get("topic_name"),
                    implemented=implemented,
                )
                return [response, self._sensor_info(tab, implemented)]
            if request.op == "get_nav_snapshot":
                return [await self._snapshot(request)]
            if (
                request.op.startswith("camera_")
                or request.op in {"get_camera_status", "get_camera_ptz_state"}
            ):
                return [await self._camera(request)]
            return [await self._dispatch_service(request)]
        except RosGatewayError as error:
            return [ack(
                request,
                ok=False,
                error=str(error),
                error_code=error.code,
            )]
        except (OSError, ValueError) as error:
            return [ack(
                request,
                ok=False,
                error=str(error),
                error_code="INVALID_REQUEST",
            )]

    async def _dispatch_service(self, request: OperatorRequest) -> dict[str, Any]:
        ros_request = build_ros_request(request)
        response = await self._call(request.op, ros_request)
        values = _message_dict(response)
        ok = bool(values.pop("ok", True))
        error = str(values.pop("error", "") or "")
        return ack(request, ok=ok, error=None if ok else error, **values)

    async def _camera(self, request: OperatorRequest) -> dict[str, Any]:
        response = await self._call(request.op, build_ros_request(request))
        values = _message_dict(response)
        success = bool(values.get("ok", values.get("success", False)))
        error = str(values.get("error", values.get("message", "")) or "")
        if not success:
            return ack(
                request,
                ok=False,
                error=error or "camera operation failed",
                payload=_camera_payload(values),
            )
        if request.op == "get_camera_status":
            return ack(request, ok=True, payload=_camera_payload(values))
        if request.op == "get_camera_ptz_state":
            return ack(request, ok=True, payload=_camera_payload(values))
        refreshed = await self._call("get_camera_ptz_state", CameraPtzState.Request())
        refreshed_values = _message_dict(refreshed)
        refreshed_ok = bool(refreshed_values.get("ok", False))
        refreshed_error = str(refreshed_values.get("error", "") or "")
        return ack(
            request,
            ok=refreshed_ok,
            error=None if refreshed_ok else refreshed_error or "camera unavailable",
            payload=_camera_payload(refreshed_values),
        )

    async def _snapshot(self, request: OperatorRequest) -> dict[str, Any]:
        response = await self._call("get_nav_snapshot", GetNavSnapshot.Request())
        if not response.ok:
            return {
                "op": "nav_snapshot",
                "ok": False,
                "error": response.error or "snapshot request failed",
                "client_req_id": request.request_id,
            }
        stamp = response.stamp
        return {
            "op": "nav_snapshot",
            "ok": True,
            "error": None,
            "mime": response.mime,
            "width": response.width,
            "height": response.height,
            "frame_id": response.frame_id,
            "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
            "layers": _message_dict(response.layers),
            "image_b64": base64.b64encode(bytes(response.image_png)).decode("ascii"),
            "image_size_bytes": len(response.image_png),
            "client_req_id": request.request_id,
        }

    async def wait_for_required_service(
        self,
        operation: str,
        request: Any,
    ) -> Any:
        """Require discovery, then one independently bounded service round-trip."""
        client = self._service_clients[operation]
        loop = asyncio.get_running_loop()
        discovery_started = loop.time()
        discovery_deadline = (
            discovery_started + self._required_service_startup_timeout_s
        )
        while not client.service_is_ready():
            if loop.time() >= discovery_deadline:
                raise RosGatewayError(
                    "SERVICE_UNAVAILABLE",
                    f"{operation} required service unavailable during startup",
                )
            await asyncio.sleep(0.05)

        # Discovery can legitimately consume most of the startup window on a
        # contended runner.  Do not give the actual readiness round-trip only
        # the leftover milliseconds: once discovered it gets the same bounded
        # response budget as a normal service call.
        discovered_at = loop.time()
        ros_future = client.call_async(request)
        future: asyncio.Future[Any] = loop.create_future()

        def complete(done: Any) -> None:
            def resolve() -> None:
                if future.done():
                    return
                try:
                    future.set_result(done.result())
                except Exception as error:
                    future.set_exception(error)

            loop.call_soon_threadsafe(resolve)

        ros_future.add_done_callback(complete)
        try:
            result = await asyncio.wait_for(future, self._service_timeout_s)
        except asyncio.TimeoutError as error:
            ros_future.cancel()
            raise RosGatewayError(
                "SERVICE_TIMEOUT",
                (
                    f"{operation} required service did not respond during startup "
                    f"within {self._service_timeout_s:.1f}s response budget"
                ),
            ) from error
        self.get_logger().info(
            f"required Cockpit service {operation} discovered in "
            f"{discovered_at - discovery_started:.3f}s and completed startup "
            f"round-trip in {loop.time() - discovered_at:.3f}s"
        )
        return result

    async def _call(self, operation: str, request: Any) -> Any:
        client = self._service_clients[operation]
        loop = asyncio.get_running_loop()
        started_at = loop.time()
        discovery_deadline = loop.time() + self._service_discovery_timeout_s
        while not client.service_is_ready():
            if loop.time() >= discovery_deadline:
                raise RosGatewayError(
                    "SERVICE_UNAVAILABLE", f"{operation} service unavailable"
                )
            await asyncio.sleep(0.05)
        ros_future = client.call_async(request)
        future: asyncio.Future[Any] = loop.create_future()

        def complete(done: Any) -> None:
            def resolve() -> None:
                if future.done():
                    return
                try:
                    future.set_result(done.result())
                except Exception as error:
                    future.set_exception(error)

            loop.call_soon_threadsafe(resolve)

        ros_future.add_done_callback(complete)
        try:
            timeout_s = (
                self._long_service_timeout_s
                if operation in {
                    "set_zones_geojson",
                    "load_zones_file",
                    "get_nav_snapshot",
                    "set_navigation_profile",
                }
                else self._service_timeout_s
            )
            result = await asyncio.wait_for(future, timeout_s)
            self.get_logger().debug(
                f"Cockpit service {operation} completed in "
                f"{loop.time() - started_at:.3f}s"
            )
            return result
        except asyncio.TimeoutError as error:
            ros_future.cancel()
            self.get_logger().warning(
                f"Cockpit service {operation} timed out after "
                f"{loop.time() - started_at:.3f}s "
                f"(response budget {timeout_s:.3f}s)"
            )
            raise RosGatewayError("SERVICE_TIMEOUT", f"{operation} service timed out") from error

    def _publish_manual(self, fields: Mapping[str, Any]) -> None:
        message = CmdVelFinal()
        message.twist.linear.x = float(fields["linear_x"])
        message.twist.angular.z = float(fields["angular_z"])
        message.brake_pct = int(fields["brake_pct"])
        message.source = CmdVelFinal.SOURCE_MANUAL
        self._teleop.publish(message)

    def _on_nav_telemetry(self, message: NavTelemetry) -> None:
        values = _message_dict(message)
        with self._lock:
            self._cache.update(values)
            mode = "idle"
            if message.manual_enabled:
                mode = "manual"
            elif message.goal_active:
                mode = "navigating"
            self._cache["mode"] = mode
        self._on_cached_telemetry_change()

    def _on_nav_event(self, message: NavEvent) -> None:
        payload = _message_dict(message)
        payload["op"] = "nav_event"
        self._emit(payload)

    def _on_capabilities(self, message: SystemCapabilities) -> None:
        values = _message_dict(message)
        capabilities = {}
        for item in values.get("capabilities", []):
            if not isinstance(item, Mapping) or not item.get("capability_id"):
                continue
            projected = dict(item)
            projected["state_label"] = capability_state_label(item.get("state"))
            capabilities[str(item["capability_id"])] = projected
        with self._lock:
            self._cache["capability_profile"] = message.profile
            self._cache["capabilities"] = capabilities
        self._on_cached_telemetry_change()

    def _on_drive_telemetry(self, message: DriveTelemetry) -> None:
        values = _message_dict(message)
        with self._lock:
            self._cache["drive_telemetry"] = values
        if self._telemetry_profile == "full":
            self._emit({"op": "drive_telemetry", **values})
        else:
            self._on_cached_telemetry_change()

    def _on_battery_guard(self, message: BatteryMissionGuard) -> None:
        with self._lock:
            self._cache.update({
                "battery_mission_state": message.state,
                "battery_return_home_recommended": message.return_home_recommended,
                "battery_recovered_voltage_v": _finite_or_none(message.recovered_voltage_v),
                "battery_loaded_voltage_v": _finite_or_none(message.loaded_voltage_slow_v),
                "battery_state": message.state,
            })
        self._on_cached_telemetry_change()

    def _on_battery_state(self, message: BatteryState) -> None:
        percentage = _finite_or_none(message.percentage)
        with self._lock:
            self._cache.update({
                "battery_pct": None if percentage is None else percentage * 100.0,
                "battery_voltage_v": _finite_or_none(message.voltage),
                "battery_present": bool(message.present),
            })
        if self._telemetry_profile == "compact":
            self._on_cached_telemetry_change()

    def _on_gps(self, message: NavSatFix) -> None:
        lat = _finite_or_none(message.latitude)
        lon = _finite_or_none(message.longitude)
        if lat is None or lon is None:
            return
        with self._lock:
            previous_pose = self._cache.get("robot_pose")
            pose = {"lat": lat, "lon": lon}
            if isinstance(previous_pose, Mapping):
                heading_deg = _finite_or_none(previous_pose.get("heading_deg"))
                if heading_deg is not None:
                    pose["heading_deg"] = heading_deg
            self._cache["robot_pose"] = pose
        if self._telemetry_profile == "full":
            self._emit({"op": "robot_pose", "pose": pose})
        else:
            self._on_cached_telemetry_change()

    def _on_heading_odometry(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        heading_deg = quaternion_yaw_deg(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        if heading_deg is None:
            return
        with self._lock:
            previous_pose = self._cache.get("robot_pose")
            if not isinstance(previous_pose, Mapping):
                return
            pose = dict(previous_pose)
            pose["heading_deg"] = heading_deg
            self._cache["robot_pose"] = pose
        self._on_cached_telemetry_change()

    def _on_rtk(self, message: String) -> None:
        status = {"raw": message.data, "available": True, "source": "rtk_status"}
        with self._lock:
            if not accepts_legacy_rtk_status(self._typed_rtk_status_received):
                return
            self._cache["gps_status"] = status
        if self._telemetry_profile == "full":
            self._emit({"op": "gps_status", "gps_status": status})
        else:
            self._on_cached_telemetry_change()

    def _on_gnss_rtk_status(self, message: GnssRtkStatus) -> None:
        status = gnss_rtk_status_payload(message)
        with self._lock:
            self._typed_rtk_status_received = True
            self._cache["gps_status"] = status
        if self._telemetry_profile == "full":
            self._emit({"op": "gps_status", "gps_status": status})
        else:
            self._on_cached_telemetry_change()

    def _on_scan_preview(self, message: LaserScan) -> None:
        payload = scan_preview_payload(message)
        if payload is not None:
            self._emit(payload)

    def _on_cached_telemetry_change(self) -> None:
        """Emit full telemetry immediately or compact transitions immediately."""
        if self._telemetry_profile == "full":
            self._emit(self._cached_payload("nav_telemetry"))
            return
        with self._lock:
            snapshot = self._compact_policy.snapshot(self._cache)
            immediate = self._compact_policy.observe(snapshot)
        if immediate:
            self._emit({"op": "nav_telemetry", **snapshot})

    def _on_compact_telemetry_timer(self) -> None:
        if self._telemetry_profile != "compact":
            return
        with self._lock:
            if not self._compact_policy.due():
                return
            self._compact_policy.mark_emitted()
            snapshot = self._compact_policy.snapshot(self._cache)
        self._emit({"op": "nav_telemetry", **snapshot})

    def _cached_payload(self, operation: str) -> dict[str, Any]:
        with self._lock:
            return {"op": operation, **deepcopy(self._cache)}

    def _sensor_info(self, tab: str, implemented: bool) -> dict[str, Any]:
        payload = {"op": "sensor_info", "tab": tab, "implemented": implemented}
        if implemented:
            with self._lock:
                payload["nodes"] = []
                payload["summary"] = {
                    "connected": self._cache.get("connected", True),
                    "mode": self._cache.get("mode", "connected"),
                }
        return payload

    def _emit(self, payload: dict[str, Any]) -> None:
        callback = self._broadcast
        if callback is not None:
            callback(payload)


def build_ros_request(request: OperatorRequest) -> Any:
    fields = request.fields
    if request.op == "camera_pan":
        result = CameraPan.Request()
        result.angle_deg = float(fields["angle"])
        return result
    if request.op == "camera_zoom_toggle":
        return Trigger.Request()
    if request.op in {"get_camera_status", "get_camera_ptz_state"}:
        if request.op == "get_camera_status":
            return CameraStatus.Request()
        return CameraPtzState.Request()
    if request.op == "camera_ptz_move":
        result = CameraPtz.Request()
        result.relative = bool(fields["relative"])
        result.apply_pan = "pan_deg" in fields
        result.pan_deg = float(fields.get("pan_deg", 0.0))
        result.apply_tilt = "tilt_deg" in fields
        result.tilt_deg = float(fields.get("tilt_deg", 0.0))
        result.apply_zoom = "zoom_level" in fields
        result.zoom_level = float(fields.get("zoom_level", 0.0))
        return result
    if request.op == "camera_ptz_preset":
        result = CameraPreset.Request()
        result.preset = fields["preset"]
        return result
    if request.op == "camera_ptz_set_preset":
        result = CameraSavePreset.Request()
        result.preset = fields["preset"]
        result.save_zoom = bool(fields["save_zoom"])
        return result
    if request.op == "set_goal_ll":
        waypoints = _waypoints(fields)
        result = SetNavGoalLL.Request()
        result.lats = [item["lat"] for item in waypoints]
        result.lons = [item["lon"] for item in waypoints]
        result.yaws_deg = [item.get("yaw_deg", math.nan) for item in waypoints]
        result.loop = bool(fields.get("loop", False))
        result.suppress_success_brake = bool(fields.get("suppress_success_brake", False))
        if len(waypoints) == 1:
            result.lat, result.lon = result.lats[0], result.lons[0]
            result.yaw_deg = result.yaws_deg[0]
        return result
    if request.op == "set_route_ll":
        waypoints = _waypoints(fields)
        result = SetRouteMissionLL.Request()
        result.lats = [item["lat"] for item in waypoints]
        result.lons = [item["lon"] for item in waypoints]
        result.yaws_deg = [item.get("yaw_deg", math.nan) for item in waypoints]
        result.waypoint_action_jsons = [json.dumps(item.get("actions", [])) for item in waypoints]
        result.waypoint_roles = [str(item.get("role", "normal")) for item in waypoints]
        result.loop = bool(fields.get("loop", False))
        _route_options(result, fields)
        return result
    if request.op == "set_patrol_ll":
        patrol = fields.get("patrol_mission")
        if not isinstance(patrol, Mapping):
            raise ValueError("patrol_mission must be an object")
        result = SetPatrolMissionLL.Request()
        _waypoint_arrays(result, "loop", _waypoints({"waypoints": patrol.get("loop_waypoints")}))
        home = _waypoints({"waypoints": [patrol.get("home_waypoint")]})[0]
        result.home_lat = home["lat"]
        result.home_lon = home["lon"]
        result.home_yaw_deg = home.get("yaw_deg", math.nan)
        _waypoint_arrays(result, "return", _optional_waypoints(patrol.get("return_waypoints")))
        _waypoint_arrays(result, "depart", _optional_waypoints(patrol.get("depart_waypoints")))
        result.depart_entry_loop_index = max(0, int(patrol.get("depart_entry_loop_index", 0)))
        _route_options(result, fields)
        return result
    constructors = {
        "cancel_goal": CancelNavGoal.Request,
        "cancel_route": CancelRouteMission.Request,
        "cancel_patrol": CancelPatrolMission.Request,
        "request_return_home": RequestReturnHome.Request,
        "load_zones_file": Trigger.Request,
    }
    if request.op in constructors:
        return constructors[request.op]()
    if request.op == "brake":
        result = BrakeNav.Request()
        result.duration_s = float(fields.get("duration_s", 1.0))
        result.brake_pct = max(0, min(100, int(fields.get("brake_pct", 100))))
        return result
    if request.op == "set_manual_mode":
        result = SetManualMode.Request()
        result.enabled = fields["enabled"]
        return result
    if request.op == "set_navigation_profile":
        result = SetNavigationProfile.Request()
        result.profile = fields["profile"]
        return result
    if request.op == "set_zones_geojson":
        geojson = fields.get("geojson")
        if geojson is None:
            raise ValueError("geojson field is required")
        result = SetZonesGeoJson.Request()
        result.geojson = geojson if isinstance(geojson, str) else json.dumps(geojson)
        return result
    raise ValueError(f"no ROS mapping for {request.op}")


def _waypoints(fields: Mapping[str, Any]) -> list[dict[str, Any]]:
    raw = fields.get("waypoints")
    if raw is None and "lat" in fields and "lon" in fields:
        raw = [fields]
    if not isinstance(raw, list) or not raw:
        raise ValueError("waypoints must be a non-empty list")
    normalized: list[dict[str, Any]] = []
    for index, item in enumerate(raw):
        if not isinstance(item, Mapping):
            raise ValueError(f"waypoint[{index}] must be an object")
        lat = _required_finite(item.get("lat", item.get("latitude")), "lat")
        lon = _required_finite(item.get("lon", item.get("longitude")), "lon")
        waypoint = {"lat": lat, "lon": lon}
        yaw = item.get("yaw_deg", item.get("yaw"))
        if yaw is not None:
            waypoint["yaw_deg"] = _required_finite(yaw, "yaw_deg")
        if "actions" in item:
            waypoint["actions"] = item["actions"]
        if "role" in item:
            waypoint["role"] = item["role"]
        normalized.append(waypoint)
    return normalized


def _optional_waypoints(raw: Any) -> list[dict[str, Any]]:
    if raw in (None, []):
        return []
    return _waypoints({"waypoints": raw})


def _waypoint_arrays(result: Any, prefix: str, waypoints: list[dict[str, Any]]) -> None:
    setattr(result, f"{prefix}_lats", [item["lat"] for item in waypoints])
    setattr(result, f"{prefix}_lons", [item["lon"] for item in waypoints])
    setattr(result, f"{prefix}_yaws_deg", [item.get("yaw_deg", math.nan) for item in waypoints])
    setattr(
        result,
        f"{prefix}_waypoint_action_jsons",
        [json.dumps(item.get("actions", [])) for item in waypoints],
    )


def _route_options(result: Any, fields: Mapping[str, Any]) -> None:
    result.leg_spacing_m = float(fields.get("leg_spacing_m", 2.0))
    result.chunk_span_m = float(fields.get("chunk_span_m", 25.0))
    result.chunk_max_waypoints = int(fields.get("chunk_max_waypoints", 20))


def _required_finite(value: Any, name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = math.nan
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def _finite_or_none(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def quaternion_yaw_deg(x: Any, y: Any, z: Any, w: Any) -> float | None:
    """Return ROS yaw in degrees, rejecting non-finite/degenerate quaternions."""
    values = tuple(_finite_or_none(value) for value in (x, y, z, w))
    if any(value is None for value in values):
        return None
    qx, qy, qz, qw = (float(value) for value in values)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-12:
        return None
    qx, qy, qz, qw = (value / norm for value in (qx, qy, qz, qw))
    sin_yaw = 2.0 * (qw * qz + qx * qy)
    cos_yaw = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.degrees(math.atan2(sin_yaw, cos_yaw))


def _camera_payload(values: Mapping[str, Any]) -> dict[str, Any]:
    """Normalize camera service output to the Cockpit's stable state shape."""
    return {
        "op": "camera_ptz_state",
        "ok": bool(values.get("ok", values.get("success", False))),
        "error": str(values.get("error", values.get("message", "")) or ""),
        "pan_deg": _finite_or_none(values.get("pan_deg")) or 0.0,
        "tilt_deg": _finite_or_none(values.get("tilt_deg")) or 0.0,
        "zoom_level": _finite_or_none(values.get("zoom_level")) or 0.0,
        "zoom_in": bool(values.get("zoom_in", False)),
        "last_command": str(values.get("last_command", "none")),
        "active_preset": str(values.get("active_preset", "")),
        "applied_preset": str(values.get("applied_preset", "")),
        "saved_preset": str(values.get("saved_preset", "")),
    }


def scan_preview_payload(message: LaserScan) -> dict[str, Any] | None:
    """Project a reduced scan without promoting it to a safety interface."""
    numeric = (
        message.angle_min,
        message.angle_increment,
        message.range_min,
        message.range_max,
    )
    if (
        not message.header.frame_id
        or not message.ranges
        or not all(math.isfinite(float(value)) for value in numeric)
        or message.angle_increment <= 0.0
        or message.range_min < 0.0
        or message.range_max <= message.range_min
    ):
        return None
    ranges = [float(value) for value in message.ranges]
    valid_count = sum(
        math.isfinite(value) and message.range_min <= value <= message.range_max
        for value in ranges
    )
    stamp = message.header.stamp
    return {
        "op": "scan_preview",
        "frame_id": message.header.frame_id,
        "stamp": {"sec": stamp.sec, "nanosec": stamp.nanosec},
        "angle_min": float(message.angle_min),
        "angle_increment": float(message.angle_increment),
        "range_min": float(message.range_min),
        "range_max": float(message.range_max),
        "ranges": ranges,
        "valid_count": valid_count,
    }


def _message_dict(message: Any) -> dict[str, Any]:
    return dict(message_to_ordereddict(message))
