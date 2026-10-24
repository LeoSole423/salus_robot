"""Shared route definitions; no navigation side effects."""
from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
import fcntl
import json
import math
import os
from pathlib import Path
import tempfile
import threading
from uuid import uuid4


class RouteError(ValueError):
    def __init__(self, code: str, message: str):
        super().__init__(message)
        self.code = code


def _invalid(message):
    raise RouteError("INVALID_ROUTE", message)


def validate_route(document):
    """Validate the editor document without silently removing route information."""
    if not isinstance(document, dict):
        _invalid("route document must be an object")
    points = document.get("waypoints")
    if not isinstance(points, list) or not 1 <= len(points) <= 200:
        _invalid("route must contain between 1 and 200 waypoints")
    home_count = 0
    local_ids = set()
    for point in points:
        if not isinstance(point, dict):
            _invalid("waypoint must be an object")
        if "localId" in point:
            local_id = point["localId"]
            if not isinstance(local_id, str) or not local_id or local_id in local_ids:
                _invalid("waypoint localId must be a unique non-empty string")
            local_ids.add(local_id)
        for key in ("x", "y", "yawDeg"):
            if key == "yawDeg" and key not in point:
                continue
            value = point.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                _invalid(f"{key} must be finite")
        # Cockpit's map editor uses x=latitude and y=longitude.
        if not -90 <= point["x"] <= 90 or not -180 <= point["y"] <= 180:
            _invalid("coordinates out of range")
        if point.get("role", "normal") not in ("normal", "home"):
            _invalid("invalid waypoint role")
        home_count += point.get("role") == "home"
        if home_count > 1:
            _invalid("only one HOME waypoint is allowed")
        actions = point.get("actions", [])
        if not isinstance(actions, list):
            _invalid("actions must be a list")
        for action in actions:
            if not isinstance(action, dict):
                _invalid("action must be an object")
            if action.get("type") == "brake_hold":
                duration = action.get("duration_s")
                brake = action.get("brake_pct", 100)
                if (type(duration) not in (int, float) or not math.isfinite(duration)
                        or not 0 < duration <= 600 or type(brake) not in (int, float)
                        or not math.isfinite(brake) or not 0 <= brake <= 100):
                    _invalid("invalid brake_hold action")
            elif action.get("type") == "set_navigation_profile":
                if action.get("profile") not in ("urban", "rural"):
                    _invalid("invalid navigation profile")
            else:
                _invalid("unsupported waypoint action")
    if not isinstance(document.get("loopRoute", True), bool):
        _invalid("loopRoute must be boolean")
    refs = document.get("patrolMissionProfileRefs")
    if refs is not None:
        if not isinstance(refs, dict):
            _invalid("patrol profile must be an object")
        for key in ("homeWaypointIndex", "departEntryWaypointIndex", "loopWaypointIndices",
                    "returnWaypointIndices", "departWaypointIndices"):
            values = refs.get(key, [] if key.endswith("Indices") else -1)
            if key.endswith("Indices"):
                if not isinstance(values, list) or len(values) != len(set(map(str, values))):
                    _invalid(f"invalid {key}")
            else:
                values = [values]
            minimum = 0 if key.endswith("Indices") else -1
            if any(type(i) is not int or not minimum <= i < len(points) for i in values):
                _invalid(f"invalid {key}")
    try:
        encoded = json.dumps(document, allow_nan=False)
    except (ValueError, TypeError) as exc:
        _invalid(str(exc))
    if len(encoded.encode()) > 512_000:
        _invalid("route document too large")
    return deepcopy(document)


class RouteRepository:
    def __init__(self, path: Path):
        self.path = path
        self._mutex = threading.RLock()

    @contextmanager
    def _locked(self):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self._mutex, self.path.with_suffix(self.path.suffix + ".lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(lock, fcntl.LOCK_UN)

    def _read(self):
        if not self.path.exists():
            return {"schema_version": 1, "routes": []}
        data = json.loads(self.path.read_text(encoding="utf-8"))
        if not isinstance(data, dict) or data.get("schema_version") != 1 or not isinstance(data.get("routes"), list):
            raise RouteError("ROUTE_STORE_INVALID", "unsupported route library schema")
        names, ids = set(), set()
        for route in data["routes"]:
            if (not isinstance(route, dict) or not isinstance(route.get("id"), str)
                    or not isinstance(route.get("name"), str) or not route["name"].strip()
                    or type(route.get("revision")) is not int or route["revision"] < 1
                    or route["id"] in ids or route["name"] in names):
                raise RouteError("ROUTE_STORE_INVALID", "invalid route library entry")
            ids.add(route["id"]); names.add(route["name"])
            validate_route(route.get("document"))
        return data

    def _write(self, data):
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.path.parent,
                                             prefix=".routes-", suffix=".tmp", delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(data, stream, ensure_ascii=False, allow_nan=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            descriptor = os.open(self.path.parent, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)

    def dispatch(self, op, fields):
        with self._locked():
            data = self._read()
            routes = data["routes"]
            if op == "list_routes":
                return {"routes": [{k: r[k] for k in ("id", "name", "revision")}
                                   for r in sorted(routes, key=lambda r: r["name"].casefold())]}
            route_id = fields.get("id")
            route = next((r for r in routes if r["id"] == route_id), None)
            if op == "get_route":
                if route is None:
                    raise RouteError("ROUTE_NOT_FOUND", "La ruta ya no existe en el robot")
                return {"route": deepcopy(route)}
            if op not in ("save_route", "delete_route"):
                raise RouteError("INVALID_REQUEST", "unsupported route operation")
            if route_id is not None:
                if route is None:
                    raise RouteError("ROUTE_NOT_FOUND", "La ruta ya no existe en el robot")
                revision = fields.get("expected_revision")
                if type(revision) is not int or revision != route["revision"]:
                    raise RouteError("ROUTE_CONFLICT", "Otro operador modificó esta ruta; volvé a cargarla o guardá una copia")
            if op == "delete_route":
                if route is None:
                    raise RouteError("ROUTE_NOT_FOUND", "missing route id")
                routes.remove(route)
                self._write(data)
                return {}
            name = fields.get("name")
            if not isinstance(name, str) or not 1 <= len(name.strip()) <= 120:
                _invalid("route name must contain 1 to 120 characters")
            name = name.strip()
            if any(r["name"] == name and r is not route for r in routes):
                raise RouteError("ROUTE_NAME_EXISTS", "Ya existe una ruta con ese nombre; elegí otro")
            document = validate_route(fields.get("document"))
            saved = {"id": route["id"] if route else str(uuid4()), "name": name,
                     "revision": route["revision"] + 1 if route else 1, "document": document}
            if route:
                routes.remove(route)
            routes.append(saved)
            self._write(data)
            return {"route": deepcopy(saved)}
