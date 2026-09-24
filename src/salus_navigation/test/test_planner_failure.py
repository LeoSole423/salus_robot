from salus_navigation.planner_failure import PlannerFailureEvidence, operator_block_reason


def test_recent_planner_failure_classifies_matching_goal_abort() -> None:
    evidence = PlannerFailureEvidence()
    evidence.begin_goal(7)
    evidence.observe_log(
        name="planner_server", message="GridBased: failed to create plan, exceeded maximum iterations",
        now_s=100.0,
    )
    assert evidence.aborted_for_no_path(epoch=7, now_s=105.0)
    assert not evidence.aborted_for_no_path(epoch=8, now_s=105.0)


def test_unrelated_or_stale_logs_do_not_classify_abort() -> None:
    evidence = PlannerFailureEvidence()
    evidence.begin_goal(3)
    evidence.observe_log(name="controller_server", message="failed to create plan", now_s=100.0)
    assert not evidence.aborted_for_no_path(epoch=3, now_s=101.0)
    evidence.observe_log(name="planner_server", message="failed to create plan", now_s=100.0)
    assert not evidence.aborted_for_no_path(epoch=3, now_s=113.0)
    evidence.begin_goal(4)
    assert not evidence.aborted_for_no_path(epoch=4, now_s=101.0)


def test_operator_message_does_not_call_generic_abort_an_obstacle() -> None:
    assert "camino seguro" in operator_block_reason("NO_VALID_PATH")
    assert "navegación se interrumpió" in operator_block_reason("NAV_ABORTED")
