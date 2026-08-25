"""Strict version-one scenario loader."""

import math
from pathlib import Path

import yaml

from .models import ExpectedTurn, GoalSpec, Pose2D, ScenarioSpec


def _keys(value, required, optional=()):
    unknown = set(value) - set(required) - set(optional)
    missing = set(required) - set(value)
    if unknown or missing:
        raise ValueError(f"invalid keys; missing={sorted(missing)}, unknown={sorted(unknown)}")


def _finite(value, name):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def load_scenario(path):
    """Load a strict scenario; unknown fields fail instead of being ignored."""

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("scenario root must be a mapping")
    _keys(data, ("schema_version", "id", "world", "spawn", "goals"))
    if data["schema_version"] != 1:
        raise ValueError("unsupported scenario schema_version")
    if data["world"] not in ("free", "empty"):
        raise ValueError("world must be a portable logical identifier")
    spawn = data["spawn"]
    _keys(spawn, ("x_m", "y_m", "yaw_rad"))
    goals = []
    if not isinstance(data["goals"], list) or not data["goals"]:
        raise ValueError("goals must be a non-empty list")
    for raw in data["goals"]:
        _keys(raw, ("id", "forward_m", "lateral_m", "yaw_offset_rad",
                         "timeout_s", "expected_turn"), ("reverse_allowed",))
        timeout = _finite(raw["timeout_s"], "timeout_s")
        if timeout <= 0:
            raise ValueError("timeout_s must be positive")
        goals.append(GoalSpec(
            goal_id=str(raw["id"]),
            forward_m=_finite(raw["forward_m"], "forward_m"),
            lateral_m=_finite(raw["lateral_m"], "lateral_m"),
            yaw_offset_rad=_finite(raw["yaw_offset_rad"], "yaw_offset_rad"),
            timeout_s=timeout,
            expected_turn=ExpectedTurn(raw["expected_turn"]),
            reverse_allowed=bool(raw.get("reverse_allowed", False)),
        ))
    if not str(data["id"]).strip() or any(not goal.goal_id.strip() for goal in goals):
        raise ValueError("scenario and goal ids must be non-empty")
    return ScenarioSpec(
        scenario_id=str(data["id"]), world=data["world"],
        spawn=Pose2D(_finite(spawn["x_m"], "x_m"),
                     _finite(spawn["y_m"], "y_m"),
                     _finite(spawn["yaw_rad"], "yaw_rad")),
        goals=tuple(goals),
    )
