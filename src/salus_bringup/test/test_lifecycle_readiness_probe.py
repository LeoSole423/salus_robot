import importlib.util
from pathlib import Path
import sys

from lifecycle_msgs.msg import State


PROBE = Path(__file__).parents[3] / "tools" / "lifecycle_readiness_probe.py"
SPEC = importlib.util.spec_from_file_location("lifecycle_readiness_probe", PROBE)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def _evidence() -> probe.LifecycleEvidence:
    return probe.LifecycleEvidence(node="/test", service="/test/get_state")


def test_active_state_is_accepted() -> None:
    evidence = _evidence()
    evidence.service_available = True
    evidence.record_state(State.PRIMARY_STATE_ACTIVE, "active", 0.2)
    evidence.finish()
    assert evidence.active
    assert evidence.failure == ""


def test_inactive_and_unconfigured_states_are_reported() -> None:
    states = (
        (State.PRIMARY_STATE_INACTIVE, "inactive"),
        (State.PRIMARY_STATE_UNCONFIGURED, "unconfigured"),
    )
    for state_id, label in states:
        evidence = _evidence()
        evidence.service_available = True
        evidence.record_state(state_id, label, 0.2)
        evidence.finish()
        assert not evidence.active
        assert label in evidence.failure


def test_absent_and_nonresponsive_services_are_distinguished() -> None:
    absent = _evidence()
    absent.finish()
    assert absent.failure == "lifecycle service unavailable"
    nonresponsive = _evidence()
    nonresponsive.service_available = True
    nonresponsive.finish()
    assert nonresponsive.failure == "lifecycle service did not respond"


def test_invalid_state_is_not_active() -> None:
    evidence = _evidence()
    evidence.service_available = True
    evidence.record_state(99, "invalid", 0.2)
    evidence.finish()
    assert not evidence.active
    assert "invalid" in evidence.failure
