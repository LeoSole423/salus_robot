from unittest.mock import Mock

import pytest

from salus_hardware.camera_backend import CameraBackendError
from salus_hardware.camera_domain import CameraLimits, PtzPose
from salus_hardware.camera_features import CameraFeaturesController


def controller():
    backend = Mock(spec=["read_state", "write_absolute", "stop", "set_light"])
    backend.read_state.return_value = PtzPose(220, 23, 1)
    return CameraFeaturesController(backend, CameraLimits()), backend


def test_sweep_centers_then_alternates_120_degrees_each_side():
    feature, backend = controller()
    feature.start(10, PtzPose(223.95, 5, 1))
    assert feature.target == PtzPose(223.95, 5, 1)
    assert feature.phase == "centering"
    backend.read_state.return_value = feature.target
    feature.tick(11)
    assert feature.target == PtzPose(343.95, 5, 1)
    backend.read_state.return_value = feature.target
    feature.tick(12)
    assert feature.target == PtzPose(103.94999999999999, 5, 1)
    assert feature.phase == "sweeping_right"
    backend.read_state.return_value = feature.target
    feature.tick(13)
    assert feature.target.pan_deg == 343.95
    assert feature.phase == "sweeping_left"
    feature.stop()
    calls = backend.write_absolute.call_count
    feature.tick(14)
    assert backend.write_absolute.call_count == calls


def test_invalid_center_rejects_without_movement():
    feature, backend = controller()
    with pytest.raises(CameraBackendError, match="cannot fit"):
        feature.start(0, PtzPose(0, 0, 1))
    backend.write_absolute.assert_not_called()


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
    assert feature.phase == "recovering"
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


def test_isolated_status_failures_do_not_cancel_or_repeat_motion():
    feature, backend = controller()
    feature.start(0)
    backend.read_state.side_effect = CameraBackendError("status timeout")
    feature.tick(1)
    feature.tick(2)
    assert feature.running
    assert feature.deadline == 120
    assert backend.write_absolute.call_count == 1
    backend.stop.assert_not_called()
    backend.read_state.side_effect = None
    backend.read_state.return_value = feature.target
    feature.tick(3)
    assert feature.phase == "moving_left"
    assert feature.status_failures == 0


def test_persistent_status_failure_stops_and_light_renewal_preserves_error():
    feature, backend = controller()
    feature.start(0)
    feature.set_light("fixed", True, 0)
    backend.read_state.side_effect = CameraBackendError("status timeout")
    feature.tick(38)
    feature.tick(39)
    feature.tick(40)
    assert feature.phase == "recovering"
    assert feature.error == "status timeout"
    backend.stop.assert_called_once()
    backend.set_light.assert_called_with("fixed", True, 60)
    feature.set_light("fixed", False, 41)
    assert feature.error == "status timeout"


def test_move_failure_stops_immediately():
    feature, backend = controller()
    feature.start(0)
    backend.read_state.return_value = feature.target
    backend.write_absolute.side_effect = CameraBackendError("move rejected")
    feature.tick(1)
    assert feature.phase == "recovering"
    assert feature.error == "move rejected"
    backend.stop.assert_called_once()


def test_light_errors_are_independent_of_other_lens_and_sweep():
    feature, backend = controller()
    backend.set_light.side_effect = CameraBackendError("lost")
    with pytest.raises(CameraBackendError):
        feature.set_light("mobile", True, 0)
    backend.set_light.side_effect = None
    feature.set_light("fixed", True, 1)
    feature.start(2)
    feature.stop()
    assert "mobile" in feature.error
    feature.set_light("mobile", False, 3)
    assert not feature.error


def test_recovery_waits_and_resumes_interrupted_target_with_bounded_budget():
    feature, backend = controller()
    feature.start(0)
    target = feature.target
    feature.tick(120)
    assert feature.phase == "recovering"
    feature.tick(124)
    assert backend.write_absolute.call_count == 1
    feature.tick(125)
    assert feature.phase == "centering"
    assert backend.write_absolute.call_args.args[0] == target
    assert feature.deadline == 245
    assert not feature.error
    # A successful restart does not reset the total budget of this sweep.
    for deadline in (245, 370):
        feature.tick(deadline)
        feature.tick(deadline + 5)
    assert feature.recovery_attempts == 3
    feature.tick(495)
    assert feature.phase == "error"
    assert "exhausted" in feature.error


@pytest.mark.parametrize("cancel", ["stop", "shutdown"])
def test_cancel_during_recovery_prevents_later_motion(cancel):
    feature, backend = controller()
    feature.start(0)
    feature.tick(120)
    getattr(feature, cancel)()
    feature.tick(130)
    assert not feature.running
    assert backend.write_absolute.call_count == 1


def test_failed_stop_prevents_automatic_recovery():
    feature, backend = controller()
    feature.start(0)
    backend.stop.side_effect = CameraBackendError("offline")
    feature.tick(120)
    feature.tick(130)
    assert feature.phase == "error"
    assert "recovery cancelled" in feature.error
    assert backend.write_absolute.call_count == 1


def test_failed_restart_exhausts_budget_without_busy_loop():
    feature, backend = controller()
    feature.start(0)
    feature.tick(120)
    backend.read_state.side_effect = CameraBackendError("offline")
    for now in (125, 130, 135):
        feature.tick(now)
    assert feature.phase == "error"
    assert feature.recovery_attempts == 3
    calls = backend.read_state.call_count
    feature.tick(200)
    assert backend.read_state.call_count == calls
