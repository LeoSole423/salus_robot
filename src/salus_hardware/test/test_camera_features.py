from unittest.mock import Mock

import pytest

from salus_hardware.camera_backend import CameraBackendError
from salus_hardware.camera_domain import CameraLimits, PtzPose
from salus_hardware.camera_features import CameraFeaturesController


def controller():
    backend = Mock(spec=["read_state", "write_absolute", "stop", "set_light"])
    backend.read_state.return_value = PtzPose(220, 23, 1)
    return CameraFeaturesController(backend, CameraLimits()), backend


def test_sweep_waits_for_left_then_traverses_right_preserving_tilt():
    feature, backend = controller()
    feature.start(10)
    left = backend.write_absolute.call_args.args[0]
    assert left == PtzPose(feature.left, 23, 1)
    feature.tick(11)
    assert backend.write_absolute.call_count == 1
    backend.read_state.return_value = left
    feature.tick(12)
    assert backend.write_absolute.call_args.args[0] == PtzPose(0, 23, 1)
    assert feature.phase == "sweeping_right"
    backend.read_state.return_value = PtzPose(0, 23, 1)
    feature.tick(13)
    assert not feature.running
    backend.stop.assert_called_once()


def test_stop_cancels_future_motion_even_if_network_stop_fails():
    feature, backend = controller()
    feature.start(0)
    backend.stop.side_effect = CameraBackendError("network lost")
    with pytest.raises(CameraBackendError):
        feature.stop()
    feature.tick(20)
    assert not feature.running
    assert backend.write_absolute.call_count == 1


def test_sweep_deadline_stops_without_claiming_completion():
    feature, backend = controller()
    feature.start(0)
    feature.tick(120)
    assert feature.phase == "error"
    assert "timed out" in feature.error
    backend.stop.assert_called_once()


def test_light_off_cancels_renewal_and_each_lens_is_independent():
    feature, backend = controller()
    assert feature.lights == {"fixed": -1, "mobile": -1}
    feature.set_light("fixed", True, 0)
    feature.set_light("mobile", True, 0)
    feature.set_light("fixed", False, 1)
    backend.set_light.reset_mock()
    feature.tick(40)
    backend.set_light.assert_called_once_with("mobile", True, 60)
    assert feature.lights == {"fixed": 0, "mobile": 1}


def test_failed_renewal_marks_unknown_and_does_not_retry_indefinitely():
    feature, backend = controller()
    feature.set_light("mobile", True, 0)
    backend.set_light.side_effect = CameraBackendError("lost")
    feature.tick(40)
    assert feature.lights["mobile"] == -1
    assert not feature.renew_at
    calls = backend.set_light.call_count
    feature.tick(80)
    assert backend.set_light.call_count == calls


def test_shutdown_stops_sweep_and_disables_lights():
    feature, backend = controller()
    feature.start(0)
    feature.set_light("fixed", True, 0)
    feature.shutdown()
    backend.stop.assert_called_once()
    backend.set_light.assert_called_with("fixed", False, 60)


def test_unsupported_backend_does_not_claim_auxiliary_capabilities():
    feature = CameraFeaturesController(None, CameraLimits())
    assert not feature.supported
    with pytest.raises(CameraBackendError):
        feature.start(0)
