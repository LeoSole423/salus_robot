from salus_web.operator_guard import OperatorControlGuard
from salus_web.protocol import parse_request, validate_request


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def test_lock_rejects_controlled_operation_but_allows_brake() -> None:
    clock = Clock()
    guard = OperatorControlGuard(
        enabled=True, heartbeat_timeout_s=5.0, initially_locked=True, clock=clock
    )
    assert guard.state().reason == "STARTUP_LOCKED"
    move = validate_request(parse_request({"op": "set_goal_ll", "client_req_id": "req-3"}))
    brake = validate_request(parse_request({"op": "brake", "client_req_id": "req-4"}))
    assert guard.authorize(move).allowed is False
    assert guard.authorize(move).error_code == "CONTROL_LOCKED"
    assert guard.authorize(brake).allowed is True


def test_unlock_heartbeat_and_expiry_are_monotonic() -> None:
    clock = Clock()
    guard = OperatorControlGuard(
        enabled=True, heartbeat_timeout_s=5.0, initially_locked=True, clock=clock
    )
    assert guard.set_locked(False).locked is False
    clock.now = 4.0
    assert guard.heartbeat().locked is False
    clock.now = 9.01
    state = guard.state()
    assert state.locked is True
    assert state.reason == "UI_HEARTBEAT_TIMEOUT"


def test_disabled_guard_never_blocks_operations() -> None:
    clock = Clock()
    guard = OperatorControlGuard(
        enabled=False, heartbeat_timeout_s=1.0, initially_locked=True, clock=clock
    )
    request = validate_request(parse_request({"op": "set_manual_mode", "enabled": True}))
    clock.now = 100.0
    assert guard.authorize(request).allowed is True
    assert guard.state().locked is False
