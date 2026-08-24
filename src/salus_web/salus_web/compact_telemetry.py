"""Pure compact telemetry timing and transition policy."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Any, Callable, Mapping


VALID_TELEMETRY_PROFILES = frozenset({"compact", "full"})

# These paths are operational state changes, not high-rate measurements.
IMMEDIATE_TRANSITION_PATHS = (
    "manual_enabled", "goal_active", "mode", "collision_stop_active",
    "nav_result_event_id", "nav_result_status", "nav_result_text",
    "failure_code", "failure_component", "route_mission.state",
    "route_mission.mission_id", "route_mission.result", "patrol_mission.phase",
    "patrol_mission.mission_id", "patrol_mission.result", "active_action",
    "navigation_profile", "home_phase", "return_home_cause",
    "battery_return_home_recommended", "drive_telemetry.estop",
    "drive_telemetry.drive_enabled",
)


def normalize_telemetry_profile(value: object) -> str:
    profile = str(value).strip().lower()
    if profile not in VALID_TELEMETRY_PROFILES:
        raise ValueError("telemetry_profile must be 'compact' or 'full'")
    return profile


def positive_rate(value: object, parameter_name: str) -> float:
    try:
        rate = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{parameter_name} must be a positive finite number") from error
    if not math.isfinite(rate) or rate <= 0.0:
        raise ValueError(f"{parameter_name} must be a positive finite number")
    return rate


def transition_signature(cache: Mapping[str, Any]) -> tuple[Any, ...]:
    """Return an immutable signature of only immediate operational changes."""
    return tuple(_freeze(_lookup(cache, path)) for path in IMMEDIATE_TRANSITION_PATHS)


def _lookup(value: Mapping[str, Any], path: str) -> Any:
    current: Any = value
    for part in path.split("."):
        if not isinstance(current, Mapping):
            return None
        current = current.get(part)
    return current


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return tuple(sorted((str(key), _freeze(item)) for key, item in value.items()))
    if isinstance(value, (list, tuple)):
        return tuple(_freeze(item) for item in value)
    return value


@dataclass
class CompactTelemetryPolicy:
    """Latest-wins gate driven by an injected monotonic clock."""

    max_hz: float
    clock: Callable[[], float]
    _last_emit_s: float | None = None
    _signature: tuple[Any, ...] | None = None
    _dirty: bool = False

    def __post_init__(self) -> None:
        if self.max_hz <= 0.0:
            raise ValueError("compact telemetry rate must be positive")

    @property
    def period_s(self) -> float:
        return 1.0 / self.max_hz

    def observe(self, cache: Mapping[str, Any]) -> bool:
        """Record the newest state and return whether it needs immediate output."""
        signature = transition_signature(cache)
        immediate = self._signature is None or signature != self._signature
        self._signature = signature
        self._dirty = True
        if immediate:
            self.mark_emitted()
        return immediate

    def due(self) -> bool:
        if not self._dirty:
            return False
        return self._last_emit_s is None or self.clock() - self._last_emit_s >= self.period_s

    def mark_emitted(self) -> None:
        self._last_emit_s = self.clock()
        self._dirty = False

    def snapshot(self, cache: Mapping[str, Any]) -> dict[str, Any]:
        """Copying belongs here so emitter callers cannot leak mutable cache."""
        return deepcopy(dict(cache))
