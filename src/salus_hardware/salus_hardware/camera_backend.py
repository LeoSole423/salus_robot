"""Camera backends isolated from ROS and preset policy."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Protocol
from urllib.error import HTTPError, URLError
from urllib.request import HTTPDigestAuthHandler, HTTPPasswordMgrWithDefaultRealm, Request, build_opener
from xml.etree import ElementTree

from .camera_domain import CameraLimits, PtzPose, normalize_pose


class CameraBackendError(RuntimeError):
    pass


class CameraBackend(Protocol):
    def read_state(self) -> PtzPose:
        ...

    def write_absolute(self, pose: PtzPose) -> PtzPose:
        ...


@dataclass
class SimCameraBackend:
    limits: CameraLimits
    initial_pose: PtzPose
    delay_s: float = 0.0
    available: bool = True

    def __post_init__(self) -> None:
        self._pose = normalize_pose(self.initial_pose, self.limits)
        self._lock = threading.Lock()

    def read_state(self) -> PtzPose:
        self._wait()
        with self._lock:
            return self._pose

    def write_absolute(self, pose: PtzPose) -> PtzPose:
        self._wait()
        with self._lock:
            self._pose = normalize_pose(pose, self.limits)
            return self._pose

    def _wait(self) -> None:
        if not self.available:
            raise CameraBackendError("camera backend unavailable")
        if self.delay_s > 0.0:
            time.sleep(self.delay_s)


@dataclass(frozen=True)
class IsapiCameraConfig:
    host: str
    port: int
    username: str
    password: str
    channel: int
    timeout_s: float

    def __post_init__(self) -> None:
        if not self.host or not self.username or not self.password:
            raise ValueError("ISAPI host, username and password are required")
        if not 1 <= self.port <= 65535 or self.channel <= 0:
            raise ValueError("ISAPI port and channel must be positive")
        if not 0.1 <= self.timeout_s <= 10.0:
            raise ValueError("ISAPI timeout must be between 0.1 and 10 seconds")


class IsapiCameraBackend:
    """Small Hikvision-style absoluteEx adapter using stdlib HTTP Digest."""

    def __init__(self, config: IsapiCameraConfig, limits: CameraLimits) -> None:
        self._config = config
        self._limits = limits
        base = f"http://{config.host}:{config.port}/ISAPI/PTZCtrl/channels/{config.channel}"
        self._url = f"{base}/absoluteEx"
        password_manager = HTTPPasswordMgrWithDefaultRealm()
        password_manager.add_password(None, self._url, config.username, config.password)
        self._opener = build_opener(HTTPDigestAuthHandler(password_manager))

    def read_state(self) -> PtzPose:
        body = self._request("GET")
        try:
            root = ElementTree.fromstring(body)
            elevation = _xml_number(root, "elevation")
            azimuth = _xml_number(root, "azimuth")
            zoom = _xml_number(root, "absoluteZoom")
        except (ElementTree.ParseError, ValueError) as error:
            raise CameraBackendError("invalid ISAPI absoluteEx response") from error
        return normalize_pose(PtzPose(azimuth, elevation, zoom), self._limits)

    def write_absolute(self, pose: PtzPose) -> PtzPose:
        target = normalize_pose(pose, self._limits)
        body = (
            '<PTZData version="2.0" xmlns="http://www.hikvision.com/ver20/XMLSchema">'
            f"<elevation>{int(round(target.tilt_deg))}</elevation>"
            f"<azimuth>{int(round(target.pan_deg))}</azimuth>"
            f"<absoluteZoom>{int(round(target.zoom_level))}</absoluteZoom>"
            "</PTZData>"
        ).encode("utf-8")
        self._request("PUT", body)
        return target

    def _request(self, method: str, body: bytes | None = None) -> bytes:
        request = Request(self._url, data=body, method=method)
        if body is not None:
            request.add_header("Content-Type", "application/xml")
        try:
            with self._opener.open(request, timeout=self._config.timeout_s) as response:
                return response.read()
        except HTTPError as error:
            raise CameraBackendError(f"ISAPI HTTP {error.code}") from error
        except (URLError, TimeoutError, OSError) as error:
            raise CameraBackendError("ISAPI request failed") from error


def _xml_number(root: ElementTree.Element, name: str) -> float:
    element = root.find(f".//{{*}}{name}")
    if element is None:
        element = root.find(f".//{name}")
    if element is None or element.text is None:
        raise ValueError(f"missing {name}")
    return float(element.text)
