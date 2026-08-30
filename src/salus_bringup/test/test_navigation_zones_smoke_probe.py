import importlib.util
from pathlib import Path
import sys


PROBE = Path(__file__).parents[3] / "tools" / "smoke_navigation_zones_sim.py"
sys.path.insert(0, str(PROBE.parent))
SPEC = importlib.util.spec_from_file_location("smoke_navigation_zones_sim", PROBE)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def ready_evidence():
    return {
        "navigation_active": True,
        "actions": {"navigate": True, "plan": True, "follow": True},
        "services": {
            "zones_set": True,
            "zones_state": True,
            "zones_reload": True,
            "nav_goal": True,
            "bt_lifecycle": True,
        },
        "odometry_progressive": True,
        "odometry_finite": True,
        "telemetry_messages": 2,
        "bt_state": "active",
        "mask_messages": 1,
        "mask_frame": "map",
        "mask_cells": 100,
    }


def test_zones_startup_requires_every_causal_dependency():
    evidence = ready_evidence()
    assert probe.startup_evidence_is_ready(evidence)

    evidence["navigation_active"] = False
    assert not probe.startup_evidence_is_ready(evidence)

    evidence = ready_evidence()
    evidence["actions"]["follow"] = False
    assert not probe.startup_evidence_is_ready(evidence)

    evidence = ready_evidence()
    evidence["services"]["zones_state"] = False
    assert not probe.startup_evidence_is_ready(evidence)


def test_zones_startup_requires_progressive_finite_odometry_and_telemetry():
    evidence = ready_evidence()
    evidence["odometry_progressive"] = False
    assert not probe.startup_evidence_is_ready(evidence)

    evidence = ready_evidence()
    evidence["odometry_finite"] = False
    assert not probe.startup_evidence_is_ready(evidence)

    evidence = ready_evidence()
    evidence["telemetry_messages"] = 1
    assert not probe.startup_evidence_is_ready(evidence)


def test_zones_startup_requires_active_bt_and_keepout_mask():
    evidence = ready_evidence()
    evidence["bt_state"] = "inactive"
    assert not probe.startup_evidence_is_ready(evidence)

    evidence = ready_evidence()
    evidence["mask_messages"] = 0
    assert not probe.startup_evidence_is_ready(evidence)

    evidence = ready_evidence()
    evidence["mask_frame"] = ""
    assert not probe.startup_evidence_is_ready(evidence)

    evidence = ready_evidence()
    evidence["mask_cells"] = 0
    assert not probe.startup_evidence_is_ready(evidence)
