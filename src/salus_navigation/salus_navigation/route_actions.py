"""Pure waypoint-action parsing and lifecycle policies."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from math import isfinite


class ActionState(str, Enum):
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class RouteAction:
    kind: str
    duration_s: float = 0.0
    brake_pct: int = 0
    profile: str = ""
    label: str = ""


@dataclass
class ActionExecution:
    waypoint_index: int
    actions: tuple[RouteAction, ...]
    state: ActionState = ActionState.PENDING
    cursor: int = 0
    started_s: float | None = None
    deadline_s: float | None = None
    error: str = ""

    @property
    def current(self) -> RouteAction | None:
        return self.actions[self.cursor] if self.cursor < len(self.actions) else None

    def start(self, now_s: float) -> RouteAction | None:
        if self.state not in (ActionState.PENDING, ActionState.RUNNING):
            return None
        action = self.current
        if action is None:
            self.state = ActionState.SUCCEEDED
            return None
        self.state = ActionState.RUNNING
        self.started_s = now_s
        self.deadline_s = now_s + action.duration_s if action.kind == "brake_hold" else None
        return action

    def complete_current(self, now_s: float) -> RouteAction | None:
        if self.state != ActionState.RUNNING:
            return None
        self.cursor += 1
        self.started_s = self.deadline_s = None
        if self.cursor >= len(self.actions):
            self.state = ActionState.SUCCEEDED
            return None
        return self.start(now_s)

    def fail(self, error: str) -> None:
        self.state, self.error = ActionState.FAILED, error

    def cancel(self, reason: str = "cancelled") -> None:
        if self.state in (ActionState.PENDING, ActionState.RUNNING):
            self.state, self.error = ActionState.CANCELLED, reason

    def remaining_s(self, now_s: float) -> float:
        return max(0.0, (self.deadline_s or now_s) - now_s)


def parse_actions(raw_text: str, waypoint_index: int) -> tuple[tuple[RouteAction, ...], str, str]:
    """Return parsed actions, canonical JSON and an error string."""
    text = str(raw_text or "").strip()
    if not text:
        return (), "", ""
    try:
        payload = json.loads(text)
    except (TypeError, ValueError) as exc:
        return (), "", f"waypoint_action_jsons[{waypoint_index}] invalid json: {exc}"
    if payload in (None, "", []):
        return (), "", ""
    if not isinstance(payload, list):
        return (), "", f"waypoint_action_jsons[{waypoint_index}] must be a JSON array"
    actions: list[RouteAction] = []
    canonical: list[dict] = []
    for action_index, item in enumerate(payload):
        if not isinstance(item, dict):
            return (), "", f"waypoint_action_jsons[{waypoint_index}][{action_index}] must be an object"
        kind = str(item.get("type", "")).strip()
        label = str(item.get("label", "") or "").strip()[:80]
        if kind == "brake_hold":
            try:
                duration = float(item.get("duration_s", 0.0))
                brake = int(float(item.get("brake_pct", 100)))
            except (TypeError, ValueError):
                return (), "", f"invalid brake_hold action at waypoint {waypoint_index}"
            if not isfinite(duration) or not 0.0 < duration <= 600.0:
                return (), "", f"brake_hold duration_s at waypoint {waypoint_index} must be > 0 and <= 600"
            brake = max(0, min(100, brake))
            actions.append(RouteAction(kind, duration, brake, label=label))
            value = {"brake_pct": brake, "duration_s": duration, "type": kind}
        elif kind == "set_navigation_profile":
            profile = str(item.get("profile", "")).strip().lower()
            if profile not in ("urban", "rural"):
                return (), "", f"set_navigation_profile at waypoint {waypoint_index} must use urban or rural"
            actions.append(RouteAction(kind, profile=profile, label=label))
            value = {"profile": profile, "type": kind}
        else:
            return (), "", f"unsupported waypoint action type: {kind or '<empty>'}"
        if label:
            value["label"] = label
        canonical.append(value)
    return tuple(actions), json.dumps(canonical, separators=(",", ":"), sort_keys=True), ""
