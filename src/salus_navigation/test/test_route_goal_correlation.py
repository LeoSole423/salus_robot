from salus_navigation.route_executor_node import terminal_nav_result_is_current


def test_old_cancel_is_ignored_while_new_goal_request_is_pending() -> None:
    assert not terminal_nav_result_is_current(
        request_pending=True,
        goal_active=False,
        result_text="cancelled",
        result_event_id=14,
        request_event_floor=10,
        last_result_event_id=10,
    )


def test_old_cancel_is_ignored_after_new_request_boundary_is_known() -> None:
    assert not terminal_nav_result_is_current(
        request_pending=False,
        goal_active=False,
        result_text="cancelled",
        result_event_id=14,
        request_event_floor=15,
        last_result_event_id=10,
    )


def test_new_terminal_result_after_request_boundary_is_consumed() -> None:
    assert terminal_nav_result_is_current(
        request_pending=False,
        goal_active=False,
        result_text="succeeded",
        result_event_id=17,
        request_event_floor=15,
        last_result_event_id=14,
    )


def test_duplicate_terminal_result_is_not_consumed_twice() -> None:
    assert not terminal_nav_result_is_current(
        request_pending=False,
        goal_active=False,
        result_text="succeeded",
        result_event_id=17,
        request_event_floor=15,
        last_result_event_id=17,
    )
