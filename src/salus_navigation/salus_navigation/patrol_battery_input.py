"""ROS-free classification of battery inputs for structured patrol.

This module deliberately decides only whether an input is trustworthy enough to
request a return.  The patrol/HOME state machine consumes the decision in a
later migration cut; keeping that boundary explicit prevents a subscriber
callback from changing mission phases or commanding motion.
"""
from __future__ import annotations

from dataclasses import dataclass
from math import isfinite


_INVALID_GUARD_STATES = frozenset({"STALE", "SUSPECT", "UNAVAILABLE"})


@dataclass(frozen=True)
class BatteryReturnDecision:
    """Result of evaluating the latest inputs for the current mission."""

    newly_latched: bool
    latched: bool
    source: str
    reason: str


@dataclass(frozen=True)
class BatteryInputSnapshot:
    """Inspectable input state, intentionally independent of ROS messages."""

    valid_guard_seen: bool
    guard_valid: bool
    guard_state: str
    guard_age_s: float | None
    soc_valid: bool
    soc_pct: float | None
    return_latched: bool


class PatrolBatteryInputPolicy:
    """Prioritise a valid mission guard over the legacy SOC fallback.

    A valid guard means ``ready`` and ``fresh`` with a supported state and a
    receipt age inside ``guard_timeout_s``.  Once such a guard has ever been
    observed, SOC remains diagnostic-only: it cannot independently trigger a
    patrol return.  A mission latch is idempotent and is not cleared merely
    because a later sample recovers.
    """

    def __init__(self, *, guard_timeout_s: float = 3.0,
                 low_battery_threshold_pct: float = 25.0) -> None:
        if not isfinite(guard_timeout_s) or guard_timeout_s <= 0.0:
            raise ValueError("guard_timeout_s must be finite and positive")
        if (not isfinite(low_battery_threshold_pct)
                or not 0.0 <= low_battery_threshold_pct <= 100.0):
            raise ValueError("low_battery_threshold_pct must be in [0, 100]")
        self._guard_timeout_s = float(guard_timeout_s)
        self._threshold_pct = float(low_battery_threshold_pct)
        self._valid_guard_seen = False
        self._guard = None
        self._soc = None
        self._return_latched = False

    def begin_mission(self) -> None:
        """Reset only the per-mission decision latch.

        Trust in a previously observed mission guard intentionally survives a
        mission replacement, so the SOC fallback cannot reappear mid-runtime.
        """
        self._return_latched = False

    def ingest_guard(self, *, ready: bool, fresh: bool, state: str,
                     return_home_recommended: bool, now_s: float) -> BatteryReturnDecision:
        self._guard = {
            "ready": bool(ready), "fresh": bool(fresh),
            "state": str(state).strip().upper(),
            "recommended": bool(return_home_recommended), "received_s": float(now_s),
        }
        return self.evaluate(now_s=now_s)

    def ingest_soc(self, *, present: bool, percentage: float,
                   now_s: float) -> BatteryReturnDecision:
        value = float(percentage)
        if isfinite(value) and 0.0 <= value <= 1.0:
            value *= 100.0
        self._soc = {
            "present": bool(present), "percentage": value,
            "received_s": float(now_s),
        }
        return self.evaluate(now_s=now_s)

    def evaluate(self, *, now_s: float) -> BatteryReturnDecision:
        """Return an idempotent recommendation using only current inputs."""
        guard_valid = self._guard_valid(now_s)
        if guard_valid:
            self._valid_guard_seen = True
            if self._guard["recommended"]:
                return self._latch("mission_guard", "guard_recommended")
            return BatteryReturnDecision(False, self._return_latched, "mission_guard",
                                         "guard_not_recommended")
        if not self._valid_guard_seen and self._soc_valid(now_s):
            if self._soc["percentage"] <= self._threshold_pct:
                return self._latch("soc_fallback", "soc_threshold")
            return BatteryReturnDecision(False, self._return_latched, "soc_fallback",
                                         "soc_above_threshold")
        return BatteryReturnDecision(False, self._return_latched, "none",
                                     "guard_unavailable" if self._valid_guard_seen
                                     else "no_valid_battery_input")

    def snapshot(self, *, now_s: float) -> BatteryInputSnapshot:
        guard_age = (None if self._guard is None else max(
            0.0, float(now_s) - self._guard["received_s"]))
        soc_pct = (None if self._soc is None else self._soc["percentage"])
        return BatteryInputSnapshot(
            valid_guard_seen=self._valid_guard_seen,
            guard_valid=self._guard_valid(now_s),
            guard_state="" if self._guard is None else self._guard["state"],
            guard_age_s=guard_age,
            soc_valid=self._soc_valid(now_s), soc_pct=soc_pct,
            return_latched=self._return_latched)

    def _latch(self, source: str, reason: str) -> BatteryReturnDecision:
        newly_latched = not self._return_latched
        self._return_latched = True
        return BatteryReturnDecision(newly_latched, True, source, reason)

    def _guard_valid(self, now_s: float) -> bool:
        if self._guard is None:
            return False
        age = float(now_s) - self._guard["received_s"]
        return (age >= 0.0 and age <= self._guard_timeout_s
                and self._guard["ready"] and self._guard["fresh"]
                and self._guard["state"] not in _INVALID_GUARD_STATES)

    def _soc_valid(self, now_s: float) -> bool:
        if self._soc is None:
            return False
        age = float(now_s) - self._soc["received_s"]
        return (age >= 0.0 and age <= self._guard_timeout_s
                and self._soc["present"] and isfinite(self._soc["percentage"])
                and 0.0 <= self._soc["percentage"] <= 100.0)
