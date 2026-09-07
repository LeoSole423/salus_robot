import pytest

from salus_hardware.camera_backend import CameraBackendError, SimCameraBackend, _xml_number
from salus_hardware.camera_domain import CameraLimits, PtzPose
from xml.etree import ElementTree


class _FakeResponse:
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False

    def read(self) -> bytes:
        return b""


class _CaptureOpener:
    def __init__(self) -> None:
        self.requests = []

    def open(self, request, timeout):
        self.requests.append((request, timeout))
        return _FakeResponse()


def test_sim_backend_is_bounded_and_switchable() -> None:
    backend = SimCameraBackend(CameraLimits(), PtzPose(0.0, 0.0, 1.0))
    assert backend.write_absolute(PtzPose(370.0, 4.0, 2.0)) == PtzPose(10.0, 4.0, 2.0)
    backend.available = False
    with pytest.raises(CameraBackendError):
        backend.read_state()


def test_xml_parser_accepts_namespaced_leaf_elements() -> None:
    root = ElementTree.fromstring('<PTZData xmlns="urn:test"><azimuth>90</azimuth></PTZData>')
    assert _xml_number(root, "azimuth") == 90.0


def test_isapi_absolute_ex_put_uses_hikvision_absolute_ex_document() -> None:
    from salus_hardware.camera_backend import IsapiCameraBackend, IsapiCameraConfig

    backend = IsapiCameraBackend(
        IsapiCameraConfig("camera.local", 80, "user", "secret", 1, 1.0),
        CameraLimits(),
    )
    opener = _CaptureOpener()
    backend._opener = opener

    backend.write_absolute(PtzPose(12.0, 3.0, 4.0))

    request, timeout = opener.requests[0]
    assert timeout == 1.0
    assert request.get_method() == "PUT"
    assert request.full_url.endswith("/ISAPI/PTZCtrl/channels/1/absoluteEx")
    assert request.get_header("Content-type") == "application/xml"

    body = request.data
    assert body is not None
    root = ElementTree.fromstring(body)
    assert root.tag.rsplit("}", 1)[-1] == "PTZAbsoluteEx"
    assert b"<PTZData" not in body
    assert _xml_number(root, "elevation") == 3.0
    assert _xml_number(root, "azimuth") == 12.0
    assert _xml_number(root, "absoluteZoom") == 4.0
