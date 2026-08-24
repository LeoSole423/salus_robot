"""ROS adapter for the camera PTZ domain and backend."""

from __future__ import annotations

import math
import os
from pathlib import Path
import threading
import time
from typing import Callable

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

from salus_interfaces.srv import (
    CameraPan,
    CameraPreset,
    CameraPtz,
    CameraPtzState,
    CameraSavePreset,
    CameraStatus,
)

from .camera_backend import (
    CameraBackend,
    CameraBackendError,
    IsapiCameraBackend,
    IsapiCameraConfig,
    SimCameraBackend,
)
from .camera_domain import (
    CameraCommandResult,
    CameraLimits,
    CameraState,
    PtzPose,
    default_presets,
    matching_preset,
    normalize_pose,
    resolve_preset,
    saved_preset,
    target_pose,
)
from .camera_presets import PresetRepository


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("salus_camera")
        self.declare_parameter("backend", "sim")
        self.declare_parameter("camera_host", "")
        # Zero means "take the operational environment value, then default";
        # it makes an explicit parameter override unambiguous.
        self.declare_parameter("camera_port", 0)
        self.declare_parameter("camera_channel", 0)
        self.declare_parameter("camera_timeout_s", 2.0)
        self.declare_parameter("camera_probe_cooldown_s", 5.0)
        self.declare_parameter("camera_presets_file", "runtime/camera/presets.json")
        self.declare_parameter("camera_pan_min_deg", 0.0)
        self.declare_parameter("camera_pan_max_deg", 355.0)
        self.declare_parameter("camera_tilt_min_deg", 0.0)
        self.declare_parameter("camera_tilt_max_deg", 90.0)
        self.declare_parameter("camera_zoom_min", 1.0)
        self.declare_parameter("camera_zoom_max", 4.0)
        self.declare_parameter("camera_zoom_zero_level", 1.0)
        self.declare_parameter("camera_zoom_fixed_level", 4.0)
        self.declare_parameter("camera_zoom_initial_in", False)
        self._limits = CameraLimits(
            float(self.get_parameter("camera_pan_min_deg").value),
            float(self.get_parameter("camera_pan_max_deg").value),
            float(self.get_parameter("camera_tilt_min_deg").value),
            float(self.get_parameter("camera_tilt_max_deg").value),
            float(self.get_parameter("camera_zoom_min").value),
            float(self.get_parameter("camera_zoom_max").value),
        )
        self._zoom_zero = _bounded_parameter(
            self, "camera_zoom_zero_level",
            self._limits.zoom_min, self._limits.zoom_max,
        )
        self._zoom_fixed = _bounded_parameter(
            self, "camera_zoom_fixed_level",
            self._limits.zoom_min, self._limits.zoom_max,
        )
        self._probe_cooldown_s = _bounded_parameter(
            self, "camera_probe_cooldown_s", 0.0, 60.0,
        )
        self._operation_lock = threading.RLock()
        self._last_probe_attempt_s = -math.inf
        self._presets = default_presets(self._limits, home_zoom=self._zoom_zero)
        presets_path = Path(str(
            self.get_parameter("camera_presets_file").value
        ))
        self._repository = PresetRepository(presets_path, self._limits)
        self._presets = self._repository.load(self._presets)
        self._backend: CameraBackend | None = None
        self._state = CameraState(
            False,
            None,
            bool(self.get_parameter("camera_zoom_initial_in").value),
            "none",
            "",
            "camera not initialized",
        )
        self._configure_backend()
        self.create_service(CameraPan, "/camara/camera_pan", self._on_pan)
        self.create_service(Trigger, "/camara/camera_zoom_toggle", self._on_zoom_toggle)
        self.create_service(CameraStatus, "/camara/camera_status", self._on_status)
        self.create_service(CameraPtz, "/camara/camera_ptz", self._on_ptz)
        self.create_service(CameraPreset, "/camara/camera_preset", self._on_preset)
        self.create_service(
            CameraSavePreset,
            "/camara/camera_save_preset",
            self._on_save_preset,
        )
        self.create_service(
            CameraPtzState,
            "/camara/camera_ptz_state",
            self._on_ptz_state,
        )

    def _configure_backend(self) -> None:
        backend_name = str(self.get_parameter("backend").value).strip().lower()
        initial = PtzPose(0.0, 0.0, self._zoom_zero)
        try:
            if backend_name == "sim":
                self._backend = SimCameraBackend(self._limits, initial)
            elif backend_name == "isapi":
                host = (
                    str(self.get_parameter("camera_host").value).strip()
                    or os.environ.get("CAMERA_HOST", "").strip()
                )
                username = os.environ.get("CAMERA_USER", "").strip()
                password = _camera_password()
                config = IsapiCameraConfig(
                    host=host,
                    port=_parameter_or_env_int(self, "camera_port", "CAMERA_PORT"),
                    username=username,
                    password=password,
                    channel=_parameter_or_env_int(self, "camera_channel", "CAMERA_CHANNEL"),
                    timeout_s=float(self.get_parameter("camera_timeout_s").value),
                )
                self._backend = IsapiCameraBackend(config, self._limits)
            else:
                raise ValueError("backend must be sim or isapi")
        except (OSError, ValueError):
            self._backend = None
            self._state = CameraState(
                False, None, False, "none", "",
                "camera configuration incomplete",
            )
            self.get_logger().warning(
                "camera backend unavailable: configuration invalid"
            )
            return
        self._probe(force=True)

    def _probe(self, *, force: bool = False) -> CameraState:
        now = time.monotonic()
        if not force and now - self._last_probe_attempt_s < self._probe_cooldown_s:
            return self._state
        self._last_probe_attempt_s = now
        if self._backend is None:
            return self._state
        try:
            pose = self._backend.read_state()
            self._state = self._state_from_pose(pose, last_command=self._state.last_command)
        except CameraBackendError:
            self._state = CameraState(
                False,
                self._state.pose,
                self._state.zoom_in,
                self._state.last_command,
                self._state.active_preset,
                "camera unavailable",
            )
        return self._state

    def _state_from_pose(self, pose: PtzPose, *, last_command: str) -> CameraState:
        pose = normalize_pose(pose, self._limits)
        return CameraState(
            True,
            pose,
            pose.zoom_level > self._zoom_zero,
            last_command,
            matching_preset(pose, self._presets),
            "",
        )

    def _execute(self, label: str, operation: Callable[[PtzPose], PtzPose]) -> CameraCommandResult:
        with self._operation_lock:
            if not self._state.available:
                self._probe()
            if self._backend is None or not self._state.available or self._state.pose is None:
                return CameraCommandResult(
                    False,
                    self._state.error or "camera unavailable",
                    self._state,
                )
            try:
                operation(self._state.pose)
                refreshed = self._backend.read_state()
                self._state = self._state_from_pose(refreshed, last_command=label)
                return CameraCommandResult(True, "", self._state)
            except (CameraBackendError, ValueError):
                self._state = CameraState(
                    False,
                    self._state.pose,
                    self._state.zoom_in,
                    self._state.last_command,
                    self._state.active_preset,
                    "camera command failed",
                )
                return CameraCommandResult(False, "camera command failed", self._state)

    def _read(self) -> CameraState:
        with self._operation_lock:
            return self._probe(force=True)

    def _on_pan(
        self,
        request: CameraPan.Request,
        response: CameraPan.Response,
    ) -> CameraPan.Response:
        result = self._execute(
            "pan",
            lambda current: self._backend_write(PtzPose(
                request.angle_deg,
                current.tilt_deg,
                current.zoom_level,
            )),
        )
        response.ok, response.error = result.ok, result.error
        response.applied_angle_deg = float(
            result.state.pose.pan_deg
            if result.ok and result.state.pose else 0.0
        )
        return response

    def _on_zoom_toggle(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        result = self._execute(
            "zoom_toggle",
            lambda current: self._backend_write(PtzPose(
                current.pan_deg,
                current.tilt_deg,
                self._zoom_zero
                if current.zoom_level > self._zoom_zero
                else self._zoom_fixed,
            )),
        )
        response.success, response.message = result.ok, result.error
        return response

    def _on_status(
        self,
        _request: CameraStatus.Request,
        response: CameraStatus.Response,
    ) -> CameraStatus.Response:
        _fill_status(response, self._read())
        return response

    def _on_ptz(
        self,
        request: CameraPtz.Request,
        response: CameraPtz.Response,
    ) -> CameraPtz.Response:
        def move(current: PtzPose) -> PtzPose:
            target = target_pose(
                current,
                self._limits,
                relative=request.relative,
                apply_pan=request.apply_pan,
                pan_deg=request.pan_deg,
                apply_tilt=request.apply_tilt,
                tilt_deg=request.tilt_deg,
                apply_zoom=request.apply_zoom,
                zoom_level=request.zoom_level,
            )
            return self._backend_write(target)

        label = "ptz_relative" if request.relative else "ptz_absolute"
        result = self._execute(label, move)
        response.ok, response.error = result.ok, result.error
        _fill_pose(response, result.state.pose)
        return response

    def _on_preset(
        self,
        request: CameraPreset.Request,
        response: CameraPreset.Response,
    ) -> CameraPreset.Response:
        try:
            name = resolve_preset(request.preset, self._presets)
            target = self._presets[name].pose
            result = self._execute(
                f"preset:{name}",
                lambda _current: self._backend_write(target),
            )
        except ValueError as error:
            result = CameraCommandResult(False, str(error), self._state)
            name = ""
        response.ok, response.error = result.ok, result.error
        response.applied_preset = name if result.ok else ""
        _fill_pose(response, result.state.pose)
        return response

    def _on_save_preset(
        self,
        request: CameraSavePreset.Request,
        response: CameraSavePreset.Response,
    ) -> CameraSavePreset.Response:
        with self._operation_lock:
            state = self._read()
            if not state.available or state.pose is None:
                response.ok, response.error = False, state.error or "camera unavailable"
                _fill_pose(response, None)
                return response
            try:
                next_preset = saved_preset(
                    request.preset,
                    state.pose,
                    self._presets,
                    self._limits,
                    save_zoom=request.save_zoom,
                )
                next_presets = dict(self._presets)
                next_presets[next_preset.name] = next_preset
                self._repository.save(next_presets)
                self._presets = next_presets
                self._state = self._state_from_pose(
                    state.pose,
                    last_command=f"save_preset:{next_preset.name}",
                )
                response.ok = True
                response.error = ""
                response.saved_preset = next_preset.name
            except (OSError, ValueError) as error:
                response.ok = False
                response.error = str(error)
                response.saved_preset = ""
            _fill_pose(response, self._state.pose)
            return response

    def _on_ptz_state(
        self,
        _request: CameraPtzState.Request,
        response: CameraPtzState.Response,
    ) -> CameraPtzState.Response:
        _fill_status(response, self._read())
        return response

    def _backend_write(self, pose: PtzPose) -> PtzPose:
        assert self._backend is not None
        return self._backend.write_absolute(normalize_pose(pose, self._limits))


def _camera_password() -> str:
    path = os.environ.get("CAMERA_PASS_FILE", "").strip()
    if path:
        return Path(path).read_text(encoding="utf-8").strip()
    return os.environ.get("CAMERA_PASS", "").strip()


def _bounded_parameter(node: Node, name: str, lower: float, upper: float) -> float:
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} outside supported range")
    return value


def _parameter_or_env_int(node: Node, parameter_name: str, environment_name: str) -> int:
    value = node.get_parameter(parameter_name).value
    default = 80 if parameter_name == "camera_port" else 1
    raw_environment = os.environ.get(environment_name, "").strip()
    if int(value) > 0:
        return int(value)
    return int(raw_environment) if raw_environment else default


def _fill_pose(response: object, pose: PtzPose | None) -> None:
    setattr(response, "pan_deg", float(pose.pan_deg if pose else 0.0))
    setattr(response, "tilt_deg", float(pose.tilt_deg if pose else 0.0))
    setattr(response, "zoom_level", float(pose.zoom_level if pose else 0.0))


def _fill_status(response: object, state: CameraState) -> None:
    setattr(response, "ok", state.available)
    setattr(response, "error", state.error)
    _fill_pose(response, state.pose)
    setattr(response, "zoom_in", state.zoom_in)
    setattr(response, "last_command", state.last_command)
    setattr(response, "active_preset", state.active_preset)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
