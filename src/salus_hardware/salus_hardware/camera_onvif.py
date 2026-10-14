"""Uniarch channel-2 ONVIF pan/tilt adapter (video remains in MediaMTX)."""

import json
from copy import deepcopy
from pathlib import Path
import os
from xml.etree import ElementTree as ET
from urllib.request import (
    Request,
    HTTPPasswordMgrWithDefaultRealm,
    HTTPDigestAuthHandler,
    build_opener,
)
from urllib.error import URLError
from .camera_backend import CameraBackendError, IsapiCameraConfig
from .camera_domain import CameraLimits, PtzPose, normalize_pose


class OnvifCameraBackend:
    PROFILE = "media_profile2_1"

    def __init__(self, config: IsapiCameraConfig, limits: CameraLimits, light_backup=None):
        if config.channel != 2:
            raise ValueError("Uniarch PTZ requires mobile channel 2")
        self.config, self.limits = config, limits
        self.light_backup = Path(light_backup) if light_backup else None
        self.url = f"http://{config.host}:{config.port}/onvif/ptz_service"
        passwords = HTTPPasswordMgrWithDefaultRealm()
        passwords.add_password(None, f"http://{config.host}:{config.port}/",
                               config.username, config.password)
        self.opener = build_opener(HTTPDigestAuthHandler(passwords))

    def _call(self, operation, content):
        body = (
            f'<s:Envelope xmlns:s="http://www.w3.org/2003/05/soap-envelope" '
            'xmlns:p="http://www.onvif.org/ver20/ptz/wsdl" '
            'xmlns:t="http://www.onvif.org/ver10/schema">'
            f"<s:Body><p:{operation}>{content}</p:{operation}></s:Body></s:Envelope>"
        ).encode()
        request = Request(
            self.url, data=body, headers={"Content-Type": "application/soap+xml; charset=utf-8"}
        )
        try:
            with self.opener.open(request, timeout=self.config.timeout_s) as response:
                root = ET.fromstring(response.read())
            if root.find(".//{*}Fault") is not None:
                raise CameraBackendError("ONVIF camera rejected command")
            return root
        except (URLError, OSError, ET.ParseError) as error:
            raise CameraBackendError("ONVIF camera request failed") from error

    def read_state(self):
        root = self._call("GetStatus", f"<p:ProfileToken>{self.PROFILE}</p:ProfileToken>")
        position = root.find(".//{*}Position/{*}PanTilt")
        try:
            x, y = float(position.attrib["x"]), float(position.attrib["y"])
            return normalize_pose(PtzPose((x + 1) * 180, (y + 1) * 45, 1), self.limits)
        except (AttributeError, KeyError, ValueError) as error:
            raise CameraBackendError("Invalid ONVIF pan/tilt position") from error

    def write_absolute(self, pose):
        target = normalize_pose(pose, self.limits)
        if target.zoom_level != 1:
            raise CameraBackendError("Uniarch zoom is not supported by this adapter")
        x = max(-1, min(0.944444418, target.pan_deg / 180 - 1))
        y = max(-1, min(1, target.tilt_deg / 45 - 1))
        self._call(
            "AbsoluteMove",
            f"<p:ProfileToken>{self.PROFILE}</p:ProfileToken>"
            f'<p:Position><t:PanTilt x="{x}" y="{y}"/></p:Position>'
            '<p:Speed><t:PanTilt x="0.15" y="0.15"/></p:Speed>',
        )
        return PtzPose((x + 1) * 180, (y + 1) * 45, 1)

    def stop(self):
        self._call(
            "Stop", f"<p:ProfileToken>{self.PROFILE}</p:ProfileToken>"
            "<p:PanTilt>true</p:PanTilt><p:Zoom>false</p:Zoom>",
        )

    def _lapi(self, method, path, payload=None):
        url = f"http://{self.config.host}:{self.config.port}{path}"
        request = Request(url, data=json.dumps(payload).encode() if payload is not None else None,
                          method=method, headers={"Content-Type": "application/json"})
        try:
            with self.opener.open(request, timeout=self.config.timeout_s) as response:
                result = json.loads(response.read())["Response"]
            if result["ResponseCode"] != 0:
                raise CameraBackendError("Uniarch rejected light command")
            return result["Data"]
        except (URLError, OSError, ValueError, KeyError, TypeError) as error:
            raise CameraBackendError("Uniarch light request failed") from error

    def set_light(self, lens, enabled, duration_s):
        if lens == "fixed":
            payload = {"Type": 1, "Action": int(enabled)}
            if enabled:
                payload.update(Mode=2, Interval=duration_s)
            self._lapi("PUT", "/LAPI/V1.0/Channels/0/System/LightCtrl", payload)
            return
        if lens != "mobile":
            raise ValueError("invalid camera light target")
        if self.light_backup is None:
            raise CameraBackendError("mobile light recovery storage unavailable")
        lamp_path = "/LAPI/V1.1/Channels/2/Image/LampCtrl/Video"
        exposure_path = "/LAPI/V1.0/Channels/2/Image/Advanced/Exposure/Video"
        try:
            if enabled and not self.light_backup.exists():
                backup = {"lamp": self._lapi("GET", lamp_path),
                          "exposure": self._lapi("GET", exposure_path)}
                self.light_backup.parent.mkdir(parents=True, exist_ok=True)
                temporary = self.light_backup.with_suffix(".tmp")
                with temporary.open("w", encoding="utf-8") as stream:
                    json.dump(backup, stream)
                    stream.flush()
                    os.fsync(stream.fileno())
                temporary.replace(self.light_backup)
            if not self.light_backup.exists():
                # No prior mutation by this adapter; avoid inventing an original profile.
                return
            backup = json.loads(self.light_backup.read_text())
            lamp, exposure = deepcopy(backup["lamp"]), deepcopy(backup["exposure"])
            if enabled:
                lamp["Type"] = 1
                lamp["LampList"][0]["Type"] = 1
                exposure["DayNight"]["Mode"] = 2
            try:
                self._lapi("PUT", lamp_path, lamp)
                self._lapi("PUT", exposure_path, exposure)
                actual_lamp = self._lapi("GET", lamp_path)
                actual_exposure = self._lapi("GET", exposure_path)
                if (actual_lamp["Type"] != lamp["Type"]
                        or actual_exposure["DayNight"] != exposure["DayNight"]):
                    raise CameraBackendError("mobile light configuration not confirmed")
            except CameraBackendError:
                if enabled:
                    self._lapi("PUT", lamp_path, backup["lamp"])
                    self._lapi("PUT", exposure_path, backup["exposure"])
                raise
            if not enabled:
                self.light_backup.unlink()
        except (OSError, ValueError, KeyError, TypeError) as error:
            raise CameraBackendError("mobile light recovery failed") from error
