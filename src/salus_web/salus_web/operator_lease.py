"""Exclusive per-connection ownership layered over the operator lock."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Hashable

from .operator_guard import OperatorControlGuard, OperatorLockState
from .protocol import OperatorRequest, is_controlled_operation


@dataclass(frozen=True)
class OperatorLeaseState:
    lock: OperatorLockState
    owner_present: bool
    requester_is_owner: bool


@dataclass(frozen=True)
class LeaseDecision:
    allowed: bool
    state: OperatorLeaseState
    error_code: str | None = None


class OperatorLease:
    """Give one WebSocket connection exclusive control without exposing its ID."""

    def __init__(self, guard: OperatorControlGuard) -> None:
        self._guard = guard
        self._owner: Hashable | None = None

    def set_locked(self, client_id: Hashable, locked: bool) -> LeaseDecision:
        self._sync_timeout()
        if not self._guard.enabled:
            return LeaseDecision(True, self.state_for(client_id))
        if locked:
            self._owner = None
            self._guard.set_locked(True)
            return LeaseDecision(True, self.state_for(client_id))
        if self._owner is not None and self._owner != client_id:
            return LeaseDecision(False, self.state_for(client_id), "CONTROL_OWNED")
        self._owner = client_id
        self._guard.set_locked(False)
        return LeaseDecision(True, self.state_for(client_id))

    def heartbeat(self, client_id: Hashable) -> LeaseDecision:
        self._sync_timeout()
        if not self._guard.enabled:
            return LeaseDecision(True, self.state_for(client_id))
        if self._owner != client_id:
            error = "CONTROL_LOCKED" if self._owner is None else "CONTROL_OWNED"
            return LeaseDecision(False, self.state_for(client_id), error)
        self._guard.heartbeat()
        self._sync_timeout()
        return LeaseDecision(True, self.state_for(client_id))

    def authorize(self, client_id: Hashable, request: OperatorRequest) -> LeaseDecision:
        self._sync_timeout()
        if not is_controlled_operation(request) or not self._guard.enabled:
            return LeaseDecision(True, self.state_for(client_id))
        if self._owner != client_id:
            error = "CONTROL_LOCKED" if self._owner is None else "CONTROL_OWNED"
            return LeaseDecision(False, self.state_for(client_id), error)
        guard_decision = self._guard.authorize(request)
        return LeaseDecision(
            guard_decision.allowed,
            self.state_for(client_id),
            guard_decision.error_code,
        )

    def disconnect(self, client_id: Hashable) -> bool:
        """Lock immediately when the controlling connection disappears."""

        self._sync_timeout()
        if self._owner != client_id:
            return False
        self._owner = None
        self._guard.force_locked("UI_CLIENT_DISCONNECTED")
        return True

    def state_for(self, client_id: Hashable) -> OperatorLeaseState:
        self._sync_timeout()
        return OperatorLeaseState(
            lock=self._guard.state(),
            owner_present=self._owner is not None,
            requester_is_owner=self._owner == client_id,
        )

    def _sync_timeout(self) -> None:
        state = self._guard.state()
        if state.locked and state.reason == "UI_HEARTBEAT_TIMEOUT":
            self._owner = None
