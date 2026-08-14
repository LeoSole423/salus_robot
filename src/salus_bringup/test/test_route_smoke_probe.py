import importlib.util
from pathlib import Path
import sys


PROBE = Path(__file__).parents[3] / "tools" / "smoke_route_executor_sim.py"
sys.path.insert(0, str(PROBE.parent))
SPEC = importlib.util.spec_from_file_location("smoke_route_executor_sim", PROBE)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def ready_evidence():
    return {
        "actions": {"navigate": True, "plan": True, "follow": True},
        "services": {"route": True, "fromLL": True, "lifecycle": True},
        "odometry_progressive": True,
        "odometry_finite": True,
        "telemetry_messages": 2,
        "bt_state": "active",
    }


def test_route_startup_requires_every_causal_dependency():
    evidence = ready_evidence()
    assert probe.startup_evidence_is_ready(evidence)

    evidence["actions"]["follow"] = False
    assert not probe.startup_evidence_is_ready(evidence)


def test_route_startup_rejects_stale_odometry_and_missing_telemetry():
    evidence = ready_evidence()
    evidence["odometry_progressive"] = False
    assert not probe.startup_evidence_is_ready(evidence)

    evidence = ready_evidence()
    evidence["telemetry_messages"] = 1
    assert not probe.startup_evidence_is_ready(evidence)


def test_route_startup_requires_active_bt_navigator():
    evidence = ready_evidence()
    evidence["bt_state"] = "inactive"
    assert not probe.startup_evidence_is_ready(evidence)
