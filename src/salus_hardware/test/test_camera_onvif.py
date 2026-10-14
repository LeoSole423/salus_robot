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


def test_fixed_light_uses_characterized_general_channel():
    b = backend()
    calls = []
    b._lapi = lambda *args: calls.append(args)
    b.set_light("fixed", True, 60)
    assert calls == [("PUT", "/LAPI/V1.0/Channels/0/System/LightCtrl",
                      {"Type": 1, "Action": 1, "Mode": 2, "Interval": 60})]


def test_mobile_light_preserves_and_recovers_profile_across_restart(tmp_path):
    from copy import deepcopy
    b = backend()
    b.light_backup = tmp_path / "backup.json"
    lamp = {"Type": 6, "LampList": [{"Type": 6}], "Mode": 0}
    exposure = {"DayNight": {"Mode": 0, "Sensitivity": 5}, "Gain": 20}
    data = {"lamp": deepcopy(lamp), "exposure": deepcopy(exposure)}

    def lapi(method, path, value=None):
        key = "lamp" if "LampCtrl" in path else "exposure"
        if method == "PUT":
            data[key] = deepcopy(value)
        return deepcopy(data[key])

    b._lapi = lapi
    b.set_light("mobile", True, 60)
    assert data["lamp"]["Type"] == 1
    assert data["exposure"]["DayNight"]["Mode"] == 2
    b.set_light("mobile", True, 60)
    # A new adapter can restore the original file, never the renewed on profile.
    restarted = backend()
    restarted.light_backup, restarted._lapi = b.light_backup, lapi
    restarted.set_light("mobile", False, 60)
    assert data == {"lamp": lamp, "exposure": exposure}
    assert not b.light_backup.exists()


def test_mobile_failed_exposure_restores_original_and_keeps_recovery(tmp_path):
    from copy import deepcopy
    b = backend()
    b.light_backup = tmp_path / "backup.json"
    original = {"lamp": {"Type": 6, "LampList": [{"Type": 6}]},
                "exposure": {"DayNight": {"Mode": 0}}}
    data = deepcopy(original)

    def lapi(method, path, value=None):
        key = "lamp" if "LampCtrl" in path else "exposure"
        if method == "PUT":
            if key == "exposure" and value["DayNight"]["Mode"] == 2:
                raise CameraBackendError("rejected")
            data[key] = deepcopy(value)
        return deepcopy(data[key])

    b._lapi = lapi
    with pytest.raises(CameraBackendError):
        b.set_light("mobile", True, 60)
    assert data == original
    assert b.light_backup.exists()
