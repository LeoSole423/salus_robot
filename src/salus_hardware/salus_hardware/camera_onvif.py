"""Uniarch channel-2 ONVIF pan/tilt adapter (video remains in MediaMTX)."""

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

    def __init__(self, config: IsapiCameraConfig, limits: CameraLimits):
        if config.channel != 2:
            raise ValueError("Uniarch PTZ requires mobile channel 2")
        self.config, self.limits = config, limits
        self.url = f"http://{config.host}:{config.port}/onvif/ptz_service"
        passwords = HTTPPasswordMgrWithDefaultRealm()
        passwords.add_password(None, self.url, config.username, config.password)
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
