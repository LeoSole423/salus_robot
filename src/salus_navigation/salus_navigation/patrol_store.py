"""Versioned, atomic patrol-mission persistence adapter."""
from __future__ import annotations

import json
import os
from pathlib import Path
from tempfile import NamedTemporaryFile

from .patrol_domain import PatrolMissionSpec, PatrolRoute, validate_mission
from .route_model import RouteWaypoint


def _point(point: RouteWaypoint) -> dict:
    return {"lat": point.lat, "lon": point.lon, "yaw_deg": point.yaw_deg,
            "input_index": point.input_index, "action_json": point.action_json,
            "map_x": point.map_x, "map_y": point.map_y}


def _route(route: PatrolRoute) -> dict:
    return {"waypoints": [_point(point) for point in route.waypoints],
            "actions": list(route.actions)}


def encode(spec: PatrolMissionSpec) -> dict:
    return {"schema_version": 1, "home": _point(spec.home), "loop": _route(spec.loop),
            "depart": _route(spec.depart), "return": _route(spec.returning),
            "depart_entry_loop_index": spec.depart_entry_loop_index,
            "leg_spacing_m": spec.leg_spacing_m, "chunk_span_m": spec.chunk_span_m,
            "chunk_max_waypoints": spec.chunk_max_waypoints}


def _decode_point(payload: object, label: str) -> RouteWaypoint:
    if not isinstance(payload, dict):
        raise ValueError(f"{label} must be an object")
    try:
        return RouteWaypoint(
            float(payload["lat"]), float(payload["lon"]), float(payload["yaw_deg"]),
            int(payload["input_index"]), action_json=str(payload.get("action_json", "")),
            map_x=None if payload.get("map_x") is None else float(payload["map_x"]),
            map_y=None if payload.get("map_y") is None else float(payload["map_y"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid {label}: {error}") from error


def _decode_route(payload: object, label: str) -> PatrolRoute:
    if not isinstance(payload, dict) or not isinstance(payload.get("waypoints"), list):
        raise ValueError(f"{label} must contain a waypoint list")
    actions = payload.get("actions")
    if not isinstance(actions, list) or not all(isinstance(action, str) for action in actions):
        raise ValueError(f"{label} actions must be a string list")
    return PatrolRoute(tuple(_decode_point(point, f"{label}[{index}]")
                             for index, point in enumerate(payload["waypoints"])),
                       tuple(actions))


def decode(payload: object) -> PatrolMissionSpec:
    """Decode a persisted mission without starting or resuming it.

    The ROS adapter owns the policy for restoring a valid definition after a
    process restart; this function deliberately has no file or ROS side
    effects so malformed runtime data cannot affect a live mission.
    """
    if not isinstance(payload, dict):
        raise ValueError("patrol document must be an object")
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported patrol document schema")
    try:
        spec = PatrolMissionSpec(
            home=_decode_point(payload["home"], "home"),
            loop=_decode_route(payload["loop"], "loop"),
            depart=_decode_route(payload["depart"], "depart"),
            returning=_decode_route(payload["return"], "return"),
            depart_entry_loop_index=int(payload["depart_entry_loop_index"]),
            leg_spacing_m=float(payload["leg_spacing_m"]),
            chunk_span_m=float(payload["chunk_span_m"]),
            chunk_max_waypoints=int(payload["chunk_max_waypoints"]),
        )
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"invalid patrol document: {error}") from error
    error = validate_mission(spec)
    if error:
        raise ValueError(error)
    return spec


def write_atomic(path: Path, spec: PatrolMissionSpec) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = json.dumps(encode(spec), indent=2, sort_keys=True) + "\n"
    with NamedTemporaryFile("w", encoding="utf-8", dir=path.parent, delete=False) as stream:
        stream.write(data); stream.flush(); os.fsync(stream.fileno()); temporary = Path(stream.name)
    try:
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)
