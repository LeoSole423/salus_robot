import pytest

from salus_hardware.camera_backend import CameraBackendError, SimCameraBackend, _xml_number
from salus_hardware.camera_domain import CameraLimits, PtzPose
from xml.etree import ElementTree


def test_sim_backend_is_bounded_and_switchable() -> None:
    backend = SimCameraBackend(CameraLimits(), PtzPose(0.0, 0.0, 1.0))
    assert backend.write_absolute(PtzPose(370.0, 4.0, 2.0)) == PtzPose(10.0, 4.0, 2.0)
    backend.available = False
    with pytest.raises(CameraBackendError):
        backend.read_state()


def test_xml_parser_accepts_namespaced_leaf_elements() -> None:
    root = ElementTree.fromstring('<PTZData xmlns="urn:test"><azimuth>90</azimuth></PTZData>')
    assert _xml_number(root, "azimuth") == 90.0
