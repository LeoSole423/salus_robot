import importlib.util
from pathlib import Path
import sys

from salus_interfaces.msg import CapabilityState, SystemCapabilities


PROBE = Path(__file__).parents[3] / "tools" / "probe_sensor_capabilities.py"
sys.path.insert(0, str(PROBE.parent))
SPEC = importlib.util.spec_from_file_location("probe_sensor_capabilities", PROBE)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def _message(imu_state: int, orientation_state: int) -> SystemCapabilities:
    message = SystemCapabilities()
    message.profile = "obstacle_detection"
    imu = CapabilityState()
    imu.capability_id = "local_motion_imu"
    imu.state = imu_state
    imu.required = True
    imu.enabled = True
    imu.source_ids = ["imu_primary"]
    orientation = CapabilityState()
    orientation.capability_id = "global_orientation"
    orientation.state = orientation_state
    orientation.required = True
    orientation.enabled = True
    orientation.source_ids = ["external_heading"]
    message.capabilities = [imu, orientation]
    return message


def test_snapshot_distinguishes_missing_capability_message() -> None:
    snapshot = probe.capability_snapshot(None)
    assert snapshot == {
        "message_received": False,
        "profile": "",
        "capabilities": {},
    }


def test_snapshot_reports_unavailable_and_stale_sources() -> None:
    snapshot = probe.capability_snapshot(
        _message(
            CapabilityState.STATE_UNAVAILABLE,
            CapabilityState.STATE_STALE,
        )
    )
    capabilities = snapshot["capabilities"]
    assert capabilities["local_motion_imu"]["state_label"] == "unavailable"
    assert capabilities["local_motion_imu"]["source_ids"] == ["imu_primary"]
    assert capabilities["global_orientation"]["state_label"] == "stale"
    assert capabilities["global_orientation"]["source_ids"] == ["external_heading"]


def test_snapshot_reports_ready_selected_sources() -> None:
    snapshot = probe.capability_snapshot(
        _message(CapabilityState.STATE_READY, CapabilityState.STATE_READY)
    )
    assert snapshot["message_received"]
    assert snapshot["profile"] == "obstacle_detection"
    assert snapshot["capabilities"]["local_motion_imu"]["state_label"] == "ready"
    assert snapshot["capabilities"]["global_orientation"]["state_label"] == "ready"
