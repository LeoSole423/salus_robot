"""Contract tests for the single-pose recovery tree.

The existing navigation smoke exercises normal Nav2 execution and terminal
cancellation, but it has no fault-injection hook for planner/controller
failures or an invalid PathHealth candidate.  These tests therefore verify the
production BT wiring and promotion ordering without introducing a second
simulation harness or claiming coverage that the current harness cannot run.
"""

from pathlib import Path
from xml.etree import ElementTree


TREE = Path(__file__).parents[1] / "config" / "navigation_core.xml"
SMOKE = Path(__file__).parents[2] / ".." / "tools" / "smoke_navigation_core_sim.py"


def _tree() -> ElementTree.Element:
    return ElementTree.parse(TREE).getroot()


def test_transient_controller_failure_has_local_wait_recovery() -> None:
    recovery = _tree().find(
        ".//RecoveryNode[@name='FollowPathRecovery']"
    )

    assert recovery is not None
    assert recovery.attrib["number_of_retries"] == "1"
    assert [child.tag for child in recovery] == ["FollowPath", "Wait"]
    assert recovery[1].attrib["wait_duration"] == "1"


def test_transient_planner_failure_has_outer_wait_and_replan_recovery() -> None:
    outer = _tree().find(
        ".//RecoveryNode[@name='NavigateRecovery']"
    )
    fallback = outer.find("ReactiveFallback[@name='RecoveryFallback']") if outer is not None else None

    assert fallback is not None
    assert [child.tag for child in fallback] == ["GoalUpdated", "Sequence"]
    wait_and_replan = fallback[1]
    assert wait_and_replan.attrib["name"] == "WaitAndReplan"
    assert wait_and_replan[0].tag == "Wait"
    assert wait_and_replan[0].attrib["wait_duration"] == "2"


def test_invalid_recovery_candidate_cannot_replace_active_path() -> None:
    compute = _tree().find(
        ".//Sequence[@name='ComputeHealthyRecoveryPath']"
    )

    assert compute is not None
    assert [child.tag for child in compute] == [
        "ComputePathToPose",
        "IsPathHealthValid",
        "CopyPath",
    ]
    assert compute[0].attrib["path"] == "{candidate_path}"
    assert compute[1].attrib["path"] == "{candidate_path}"
    assert compute[1].attrib["context"] == "1"
    assert compute[1].attrib["expected_state"] == "0"
    assert compute[2].attrib["input_path"] == "{candidate_path}"
    assert compute[2].attrib["output_path"] == "{path}"


def test_no_healthy_path_keeps_stop_and_wait_and_cancellation_has_no_motion_path() -> None:
    root = _tree()
    stop_and_wait = root.find(
        ".//ReactiveSequence[@name='StopAndWaitForPathData']"
    )
    local_recovery = root.find(
        ".//RecoveryNode[@name='FollowPathRecovery']"
    )
    outer_fallback = root.find(
        ".//ReactiveFallback[@name='RecoveryFallback']"
    )

    assert stop_and_wait is not None
    assert stop_and_wait[-1].tag == "AlwaysFailure"
    assert local_recovery is not None
    assert outer_fallback is not None
    assert outer_fallback[0].tag == "GoalUpdated"
    assert "Spin" not in TREE.read_text(encoding="utf-8")
    assert "BackUp" not in TREE.read_text(encoding="utf-8")


def test_existing_smoke_covers_terminal_cancel_and_safe_zero_boundary() -> None:
    smoke = SMOKE.resolve().read_text(encoding="utf-8")

    assert 'CancelNavGoal.Request()' in smoke
    assert 'not get_state(node).goal_active' in smoke
    assert 'message.twist.linear.x == 0.0 and message.brake_pct == 100' in smoke
