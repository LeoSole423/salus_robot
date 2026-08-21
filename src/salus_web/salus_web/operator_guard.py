"""Pure operator-lock policy for the future WebSocket transport."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from .protocol import OperatorRequest, is_controlled_operation


@dataclass(frozen=True)
class OperatorLockState:
    locked: bool
    reason: str
    heartbeat_age_s: float | None


@dataclass(frozen=True)
class GuardDecision:
    allowed: bool
    state: OperatorLockState
    error_code: str | None = None


class OperatorControlGuard:
    """Single-operator lock with an injected monotonic clock.

    It intentionally does not attach a client identity. Multi-client lease
    ownership needs a separate ADR before the WebSocket server is written.
    """

    def __init__(
        self,
        *,
        enabled: bool,
        heartbeat_timeout_s: float,
        initially_locked: bool,
        clock: Callable[[], float],
    ) -> None:
        if heartbeat_timeout_s <= 0.0:
            raise ValueError("heartbeat_timeout_s must be positive")
        self._enabled = enabled
        self._timeout_s = heartbeat_timeout_s
        self._clock = clock
        self._locked = initially_locked if enabled else False
        self._reason = "STARTUP_LOCKED" if self._locked else ""
        self._last_heartbeat = None if self._locked else clock()

    @property
    def enabled(self) -> bool:
        return self._enabled

    def force_locked(self, reason: str) -> OperatorLockState:
        """Enter a safe lock for a transport-level failure."""

        if not self._enabled:
            return self.state()
        self._locked = True
        self._reason = reason
        self._last_heartbeat = None
        return self.state()

    def set_locked(self, locked: bool) -> OperatorLockState:
        if not self._enabled:
            return self.state()
        self._locked = locked
        self._reason = "UI_LOCK_REQUEST" if locked else ""
        self._last_heartbeat = None if locked else self._clock()
        return self.state()

    def heartbeat(self) -> OperatorLockState:
        self._expire_if_needed()
        if self._enabled and not self._locked:
            self._last_heartbeat = self._clock()
        return self.state()

    def authorize(self, request: OperatorRequest) -> GuardDecision:
        state = self.state()
        if state.locked and is_controlled_operation(request):
            return GuardDecision(False, state, "CONTROL_LOCKED")
        return GuardDecision(True, state)

    def state(self) -> OperatorLockState:
        self._expire_if_needed()
        if not self._enabled:
            return OperatorLockState(False, "", None)
        age = None
        if self._last_heartbeat is not None:
            age = max(0.0, self._clock() - self._last_heartbeat)
        return OperatorLockState(self._locked, self._reason, age)

    def _expire_if_needed(self) -> None:
        if not self._enabled or self._locked or self._last_heartbeat is None:
            return
        if self._clock() - self._last_heartbeat > self._timeout_s:
            self._locked = True
            self._reason = "UI_HEARTBEAT_TIMEOUT"
            self._last_heartbeat = None
