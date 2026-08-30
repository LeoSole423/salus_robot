import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus

from salus_interfaces.srv import GetNavState
from salus_navigation.nav_command_server import NavCommandServer


class FakeHandle:
    accepted = True

    def __init__(self, on_cancel=None):
        self.cancel_calls = 0
        self.on_cancel = on_cancel
        self.result_callbacks = []

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.on_cancel is not None:
            self.on_cancel()
        return SimpleNamespace()

    def get_result_async(self):
        return SimpleNamespace(add_done_callback=self.result_callbacks.append)


class Future:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value


def make_server(*, handle=None, pending=False, manual=False, timeout_s=0.01):
    server = object.__new__(NavCommandServer)
    server._lock = threading.Lock()
    server._goal_epoch = 3
    server._goal_pending = pending
    server._goal_handle = handle
    server._goal_cancel_requested = False
    server._goal_cancel_reason = ""
    server._goal_terminal_event = threading.Event()
    if not pending and handle is None:
        server._goal_terminal_event.set()
    server._goal_result_status = GoalStatus.STATUS_UNKNOWN
    server._goal_result_text = "navigating" if handle is not None else "sending navigation goal"
    server._goal_result_event_id = 10
    server._suppress_success_brake = False
    server._cancel_result_timeout_s = timeout_s
    server._arbiter = SimpleNamespace(
        manual_enabled=manual,
        manual_command=None,
    )
    server._last_fix = None
    server._last_safe = None
    server._publish = lambda _message: None
    server._start_brake_hold = lambda _duration, _pct: None
    event_ids = iter(range(14, 1000))
    server._event = lambda *_args, **_kwargs: next(event_ids)
    server.get_logger = lambda: SimpleNamespace(error=lambda _message: None)
    server._goal_in_keepout = lambda _x, _y: False
    server._navigate_client = SimpleNamespace(server_is_ready=lambda: True)
    return server


def terminal_future(status):
    return Future(SimpleNamespace(status=status))


def test_get_state_returns_terminal_result_atomically() -> None:
    server = make_server(handle=FakeHandle())
    server._on_goal_result(terminal_future(GoalStatus.STATUS_SUCCEEDED), 3)

    response = NavCommandServer._on_get_state(
        server,
        GetNavState.Request(),
        GetNavState.Response(),
    )

    assert response.ok
    assert not response.goal_active
    assert response.nav_result_status == GoalStatus.STATUS_SUCCEEDED
    assert response.nav_result_text == "succeeded"
    assert response.nav_result_event_id == 14


def test_cancel_keeps_goal_active_until_terminal_result() -> None:
    handle = FakeHandle()
    server = make_server(handle=handle)

    was_active, completed = server._cancel_goal(
        "cancelled by service", apply_brake=False, wait_terminal=False
    )

    assert was_active
    assert not completed
    assert handle.cancel_calls == 1
    assert server._goal_active_locked()
    assert server._goal_handle is handle
    assert server._goal_cancel_requested
    assert server._goal_result_text == "cancelling: cancelled by service"

    server._on_goal_result(terminal_future(GoalStatus.STATUS_CANCELED), 3)

    assert not server._goal_active_locked()
    assert server._goal_handle is None
    assert not server._goal_cancel_requested
    assert server._goal_terminal_event.is_set()
    assert server._goal_result_text == "cancelled"


def test_replacement_dispatches_only_after_previous_goal_is_terminal() -> None:
    server = make_server()
    handle = FakeHandle(
        on_cancel=lambda: server._on_goal_result(
            terminal_future(GoalStatus.STATUS_CANCELED), 3
        )
    )
    server._goal_handle = handle
    server._goal_terminal_event.clear()
    dispatched = []
    server._send_map_goal = lambda point, yaw, epoch: dispatched.append(
        (point.x, point.y, yaw, epoch)
    )

    error = server._request_map_goal(
        4.0, -2.0, 15.0, suppress_success_brake=False
    )

    assert error == ""
    assert handle.cancel_calls == 1
    assert dispatched == [(4.0, -2.0, 15.0, 4)]
    assert server._goal_pending
    assert server._goal_epoch == 4
    assert not server._goal_terminal_event.is_set()


def test_replacement_is_rejected_if_previous_cancel_does_not_finish() -> None:
    handle = FakeHandle()
    server = make_server(handle=handle, timeout_s=0.001)
    dispatched = []
    server._send_map_goal = lambda *args: dispatched.append(args)

    error = server._request_map_goal(
        4.0, -2.0, 15.0, suppress_success_brake=False
    )

    assert error.startswith(
        "previous navigation goal cancellation did not reach a terminal state"
    )
    assert handle.cancel_calls == 1
    assert not dispatched
    assert server._goal_active_locked()
    assert server._goal_cancel_requested


def test_pending_goal_is_cancelled_when_acceptance_arrives_during_manual_takeover() -> None:
    server = make_server(pending=True, manual=True)
    was_active, completed = server._cancel_goal(
        "manual takeover", apply_brake=False, wait_terminal=False
    )
    assert was_active
    assert not completed

    handle = FakeHandle()
    server._on_goal_response(Future(handle), 3)

    assert server._goal_handle is handle
    assert server._goal_active_locked()
    assert handle.cancel_calls == 1
    assert len(handle.result_callbacks) == 1

    handle.result_callbacks[0](
        terminal_future(GoalStatus.STATUS_CANCELED)
    )

    assert not server._goal_active_locked()
    assert server._goal_terminal_event.is_set()


def test_terminal_result_event_id_does_not_advance_on_unrelated_events() -> None:
    server = make_server(handle=FakeHandle())

    server._on_goal_result(terminal_future(GoalStatus.STATUS_CANCELED), 3)

    assert server._goal_result_text == "cancelled"
    assert server._goal_result_event_id == 14

    unrelated_event_id = server._event(
        0, "UNRELATED_EVENT", "must not re-identify the old terminal result"
    )
    assert unrelated_event_id == 15
    assert server._goal_result_event_id == 14
