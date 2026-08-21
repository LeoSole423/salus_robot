"""Validated, atomic persistence for Cockpit waypoint YAML files."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping

import yaml


class WaypointValidationError(ValueError):
    pass


@dataclass(frozen=True)
class WaypointDocument:
    waypoints: tuple[dict[str, Any], ...]
    patrol_profile: dict[str, Any] | None


def normalize_document(raw: Mapping[str, Any]) -> WaypointDocument:
    if not isinstance(raw, Mapping):
        raise WaypointValidationError("yaml root must be a map/object")
    raw_waypoints = raw.get("waypoints")
    if not isinstance(raw_waypoints, list) or not raw_waypoints:
        raise WaypointValidationError("waypoints must be a non-empty list")

    waypoints: list[dict[str, Any]] = []
    home_count = 0
    for index, raw_waypoint in enumerate(raw_waypoints):
        waypoint = _normalize_waypoint(raw_waypoint, index)
        home_count += waypoint.get("role") == "home"
        if home_count > 1:
            raise WaypointValidationError("only one HOME waypoint is allowed")
        waypoints.append(waypoint)
    profile = _normalize_patrol_profile(raw.get("patrol_profile"), len(waypoints))
    return WaypointDocument(tuple(waypoints), profile)


def parse_yaml(text: str) -> WaypointDocument:
    try:
        raw = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise WaypointValidationError(f"invalid yaml: {exc}") from exc
    return normalize_document(raw)


def to_yaml_document(document: WaypointDocument) -> dict[str, Any]:
    result: dict[str, Any] = {"waypoints": []}
    for waypoint in document.waypoints:
        entry = {"latitude": waypoint["lat"], "longitude": waypoint["lon"]}
        if "yaw_deg" in waypoint:
            entry["yaw"] = waypoint["yaw_deg"]
        if waypoint.get("actions"):
            entry["actions"] = deepcopy(waypoint["actions"])
        if waypoint.get("role") == "home":
            entry["role"] = "home"
        result["waypoints"].append(entry)
    if document.patrol_profile is not None:
        result["patrol_profile"] = deepcopy(document.patrol_profile)
    return result


class AtomicWaypointRepository:
    """A file repository that either replaces a whole validated document or none."""

    def __init__(self, path: Path) -> None:
        self._path = Path(path)

    @property
    def path(self) -> Path:
        return self._path

    def load(self) -> WaypointDocument:
        try:
            text = self._path.read_text(encoding="utf-8")
        except FileNotFoundError:
            raise
        except OSError as exc:
            raise OSError(f"failed reading waypoints file: {exc}") from exc
        return parse_yaml(text)

    def save(self, document: WaypointDocument) -> None:
        # Revalidate before serialization so callers cannot persist a forged dataclass.
        normalized = normalize_document(
            {
                "waypoints": list(document.waypoints),
                "patrol_profile": document.patrol_profile,
            }
        )
        data = yaml.safe_dump(to_yaml_document(normalized), sort_keys=False, allow_unicode=True)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path: Path | None = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=self._path.parent,
                prefix=f".{self._path.name}.",
                suffix=".tmp",
                delete=False,
            ) as handle:
                temporary_path = Path(handle.name)
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary_path, self._path)
            _fsync_directory(self._path.parent)
        except OSError as exc:
            raise OSError(f"failed writing waypoints file: {exc}") from exc
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()


def _normalize_waypoint(raw: Any, index: int) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise WaypointValidationError(f"waypoint[{index}] must be an object")
    latitude = _finite(raw.get("lat", raw.get("latitude")), f"waypoint[{index}] latitude")
    longitude = _finite(raw.get("lon", raw.get("longitude")), f"waypoint[{index}] longitude")
    if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
        raise WaypointValidationError(f"waypoint[{index}] coordinates out of range")
    result: dict[str, Any] = {"lat": latitude, "lon": longitude}
    yaw = raw.get("yaw_deg", raw.get("yaw"))
    if yaw is not None:
        result["yaw_deg"] = _finite(yaw, f"waypoint[{index}] yaw")
    role = raw.get("role", raw.get("waypoint_role", "normal"))
    if not isinstance(role, str) or role.strip().lower() not in {"normal", "home"}:
        raise WaypointValidationError(f"invalid waypoint[{index}] role")
    if role.strip().lower() == "home":
        result["role"] = "home"
    if "actions" in raw and raw["actions"] is not None:
        if not isinstance(raw["actions"], list):
            raise WaypointValidationError(f"invalid waypoint[{index}] actions")
        result["actions"] = deepcopy(raw["actions"])
    return result


def _normalize_patrol_profile(raw: Any, waypoint_count: int) -> dict[str, Any] | None:
    if raw is None:
        return None
    if not isinstance(raw, Mapping):
        raise WaypointValidationError("patrol_profile must be an object")
    result: dict[str, Any] = {}
    for name in ("home_waypoint_index", "depart_entry_waypoint_index"):
        value = _index(raw.get(name, -1), name)
        result[name] = value if value < waypoint_count else -1
    for name in ("loop_waypoint_indices", "return_waypoint_indices", "depart_waypoint_indices"):
        value = raw.get(name, [])
        if not isinstance(value, list):
            raise WaypointValidationError(f"{name} must be a list")
        indices: list[int] = []
        for item in value:
            index = _index(item, name)
            if 0 <= index < waypoint_count and index not in indices:
                indices.append(index)
        result[name] = indices
    return result


def _finite(value: Any, name: str) -> float:
    if isinstance(value, bool):
        value = None
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = math.nan
    if not math.isfinite(number):
        raise WaypointValidationError(f"invalid {name}")
    return number


def _index(value: Any, name: str) -> int:
    if isinstance(value, bool):
        raise WaypointValidationError(f"{name} must contain integers")
    try:
        number = int(value)
    except (TypeError, ValueError) as exc:
        raise WaypointValidationError(f"{name} must contain integers") from exc
    if str(number) != str(value).strip() and not isinstance(value, int):
        raise WaypointValidationError(f"{name} must contain integers")
    return number


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
