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


def _goal(*, initial_x=0.5, positive_command=False, result=True):
    return [
        {
            "code": "ROUTE_CHUNK_DISPATCHED",
            "timestamp_s": 10.0,
            "received_monotonic_s": 20.0,
            "details": {"chunk_id": "home", "poses_xy": [[0.0, 0.0]]},
        },
        {
            "code": "GOAL_ACCEPTED",
            "timestamp_s": 10.0,
            "received_monotonic_s": 20.01,
            "details": {"goal_generation": 6},
        },
        {
            "code": "GOAL_RESULT_SUCCEEDED" if result else "GOAL_RESULT_ABORTED",
            "timestamp_s": 10.2,
            "received_monotonic_s": 20.2,
            "details": {"goal_generation": 6},
        },
    ], [
        {"stamp_s": 10.0, "x": initial_x, "y": 0.0},
        {"stamp_s": 10.1, "x": initial_x, "y": 0.0},
        {"stamp_s": 10.2, "x": initial_x, "y": 0.0},
    ], [
        {
            "received_monotonic_s": 20.1,
            "valid": True,
            "requested_linear_x_mps": 0.5 if positive_command else 0.0,
        },
    ]


def _pathology_evidence(*, initial_x=0.5, positive_command=False, result=True,
                        controller_status=True):
    events, odometry, status = _goal(
        initial_x=initial_x, positive_command=positive_command, result=result)
    if not controller_status:
        status = []
    plan = _path([
        (0.5, 0.0), (2.0, 2.0), (0.0, 2.0), (2.0, 0.0),
    ], stamp_ns=10_100_000_000)
    chunks = [_path([(0.0, 0.0)], stamp_ns=9_000_000_000)]
    return plan, chunks, events, odometry, status


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


def test_pathology_with_complete_tolerance_evidence_is_diagnostic_only():
    plan, chunks, events, odometry, status = _pathology_evidence()
    observations = probe.assert_plan_topology(
        [plan], chunks, events=events, odometry=odometry,
        controller_status=status)

    assert observations[0]["topology_ok"] is False
    assert observations[0]["gate_ok"] is True
    assert observations[0]["diagnostic_anomaly"] is True
    assert observations[0]["execution"]["classification"] == (
        "UNEXECUTED_WITHIN_TOLERANCE")


def test_pathology_with_positive_controller_command_still_fails():
    plan, chunks, events, odometry, status = _pathology_evidence(
        positive_command=True)
    try:
        probe.assert_plan_topology(
            [plan], chunks, events=events, odometry=odometry,
            controller_status=status)
    except RuntimeError as exc:
        assert "unnecessary planner loop" in str(exc)
    else:
        assert False, "a plan with a positive controller command was ignored"


def test_pathology_with_incomplete_execution_evidence_still_fails():
    plan, chunks, events, odometry, status = _pathology_evidence(
        controller_status=False)
    try:
        probe.assert_plan_topology(
            [plan], chunks, events=events, odometry=odometry,
            controller_status=status)
    except RuntimeError as exc:
        assert "unnecessary planner loop" in str(exc)
    else:
        assert False, "incomplete execution evidence was treated as a pass"
