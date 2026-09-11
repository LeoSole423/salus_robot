import importlib.util
from pathlib import Path
import sys


PROBE = Path(__file__).parents[3] / "tools" / "smoke_patrol_battery_sim.py"
sys.path.insert(0, str(PROBE.parent))
SPEC = importlib.util.spec_from_file_location("smoke_patrol_battery_sim", PROBE)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def _path(points, stamp_ns=1):
    return {"stamp_ns": stamp_ns, "frame_id": "map", "points": points}


def test_patrol_plan_topology_accepts_a_direct_chunk_path():
    observations = probe.assert_plan_topology(
        [_path([(0.0, 0.0), (4.0, 0.0), (8.0, 0.0)], stamp_ns=2)],
        [_path([(6.0, 0.0), (8.0, 0.0)])],
    )

    assert len(observations) == 1
    assert observations[0]["topology_ok"]
    assert observations[0]["self_intersections"] == 0


def test_patrol_plan_topology_rejects_a_self_intersecting_loop():
    try:
        probe.assert_plan_topology(
            [_path([(0.0, 0.0), (4.0, 4.0), (0.0, 4.0), (4.0, 0.0)], stamp_ns=2)],
            [_path([(4.0, 0.0)])],
        )
    except RuntimeError as exc:
        assert "unnecessary planner loop" in str(exc)
    else:
        assert False, "self-intersecting planner path was accepted"


def test_patrol_plan_topology_rejects_a_large_nonintersecting_detour():
    try:
        probe.assert_plan_topology(
            [_path([
                (0.0, 0.0), (0.0, 4.0), (4.0, 4.0), (4.0, -4.0),
                (0.0, -4.0), (0.0, 0.0), (8.0, 0.0),
            ], stamp_ns=2)],
            [_path([(8.0, 0.0)])],
        )
    except RuntimeError as exc:
        assert "unnecessary planner loop" in str(exc)
    else:
        assert False, "large planner detour was accepted"
