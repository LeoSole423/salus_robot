from xml.etree import ElementTree as ET
import pytest
from salus_hardware.camera_backend import IsapiCameraConfig, CameraBackendError
from salus_hardware.camera_domain import CameraLimits, PtzPose
from salus_hardware.camera_onvif import OnvifCameraBackend


def backend():
    return OnvifCameraBackend(
        IsapiCameraConfig("camera", 80, "user", "secret", 2, 2), CameraLimits()
    )


def test_status_maps_normalized_pan_tilt_to_contract():
    b = backend()
    b._call = lambda *args: ET.fromstring(
        '<Status><Position><PanTilt x="0.2" y="-0.5"/></Position></Status>'
    )
    assert b.read_state() == PtzPose(216, 22.5, 1)


def test_mobile_profile_and_bounded_absolute_command_exclude_zoom():
    b = backend()
    calls = []
    b._call = lambda *args: calls.append(args)
    pose = b.write_absolute(PtzPose(355, 90, 1))
    assert pose.pan_deg == pytest.approx(349.99999524)
    assert calls[0][0] == "AbsoluteMove"
    assert "media_profile2_1" in calls[0][1]
    assert "Zoom" not in calls[0][1]
    assert 'x="0.15" y="0.15"' in calls[0][1]
    with pytest.raises(CameraBackendError):
        b.write_absolute(PtzPose(0, 0, 2))


def test_fixed_lens_cannot_be_used_for_ptz():
    with pytest.raises(ValueError):
        OnvifCameraBackend(IsapiCameraConfig("camera", 80, "user", "secret", 1, 2), CameraLimits())


def test_malformed_status_fails_explicitly():
    b = backend()
    b._call = lambda *args: ET.fromstring("<Status/>")
    with pytest.raises(CameraBackendError):
        b.read_state()
