from salus_web.operator_guard import OperatorControlGuard
from salus_web.operator_lease import OperatorLease
from salus_web.protocol import parse_request, validate_request


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def _lease(clock: Clock) -> OperatorLease:
    return OperatorLease(OperatorControlGuard(
        enabled=True,
        heartbeat_timeout_s=2.5,
        initially_locked=True,
        clock=clock,
    ))


def _goal():
    return validate_request(parse_request({"op": "set_goal_ll"}))


def _brake():
    return validate_request(parse_request({"op": "brake"}))


def test_only_unlocking_connection_controls_motion() -> None:
    lease = _lease(Clock())
    assert lease.set_locked("client-a", False).allowed is True
    assert lease.authorize("client-a", _goal()).allowed is True
    rejected = lease.authorize("client-b", _goal())
    assert rejected.allowed is False
    assert rejected.error_code == "CONTROL_OWNED"
    assert lease.authorize("client-b", _brake()).allowed is True


def test_any_client_can_lock_but_only_owner_can_heartbeat() -> None:
    lease = _lease(Clock())
    lease.set_locked("client-a", False)
    assert lease.heartbeat("client-b").error_code == "CONTROL_OWNED"
    assert lease.set_locked("client-b", True).allowed is True
    state = lease.state_for("client-a")
    assert state.lock.locked is True
    assert state.owner_present is False


def test_disconnect_and_timeout_fail_safe_and_release_owner() -> None:
    clock = Clock()
    lease = _lease(clock)
    lease.set_locked("client-a", False)
    assert lease.disconnect("client-a") is True
    assert lease.state_for("client-a").lock.reason == "UI_CLIENT_DISCONNECTED"

    lease.set_locked("client-b", False)
    clock.now = 2.6
    state = lease.state_for("client-b")
    assert state.lock.reason == "UI_HEARTBEAT_TIMEOUT"
    assert state.owner_present is False


def test_non_owner_disconnect_does_not_change_lease() -> None:
    lease = _lease(Clock())
    lease.set_locked("client-a", False)
    assert lease.disconnect("client-b") is False
    assert lease.state_for("client-a").requester_is_owner is True
