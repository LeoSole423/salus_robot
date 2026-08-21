"""Pure codec and validation helpers for the Cockpit WebSocket contract.

This module deliberately knows nothing about rclpy, sockets, or client
ownership.  It gives the future transport layer one normalized request and a
stable way to construct correlated replies.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Any, Mapping


REQUEST_ID_ALIASES = ("client_req_id", "requestId", "clientReqId", "request_id")

KNOWN_OPERATIONS = frozenset(
    {
        "get_state", "set_control_lock", "control_heartbeat", "set_zones_geojson",
        "load_zones_file", "save_waypoints_file", "load_waypoints_file", "set_goal_ll",
        "cancel_goal", "brake", "set_route_ll", "cancel_route", "set_patrol_ll",
        "cancel_patrol", "request_return_home", "set_navigation_profile",
        "set_manual_mode", "set_manual_cmd", "get_nav_snapshot", "set_sensor_info_view",
    }
)
FIXED_DATUM_MUTATIONS = frozenset(
    {"set_datum", "save_datum", "delete_datum", "select_datum", "capture_current_gps_datum"}
)


@dataclass(frozen=True)
class ProtocolError(Exception):
    """A bounded protocol failure that can be returned to Cockpit."""

    code: str
    message: str
    request: str = "invalid_request"
    request_id: str | None = None

    def __str__(self) -> str:
        return self.message


@dataclass(frozen=True)
class OperatorRequest:
    """A normalized incoming request with top-level and payload fields merged."""

    op: str
    request_id: str | None
    fields: Mapping[str, Any]


def parse_request(raw: str | bytes | Mapping[str, Any]) -> OperatorRequest:
    """Decode an incoming JSON request without performing any ROS operation."""

    if isinstance(raw, bytes):
        try:
            raw = raw.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise ProtocolError("invalid_json", "request is not UTF-8", "invalid_json") from exc
    if isinstance(raw, str):
        try:
            value = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ProtocolError("invalid_json", "invalid JSON request", "invalid_json") from exc
    else:
        value = raw

    if not isinstance(value, Mapping):
        raise ProtocolError("invalid_request", "request must be a JSON object")
    op = value.get("op")
    if not isinstance(op, str) or not op.strip():
        raise ProtocolError("invalid_request", "request op must be a non-empty string")
    op = op.strip()
    request_id = _request_id(value, op)

    payload = value.get("payload", {})
    if payload is not None and not isinstance(payload, Mapping):
        raise ProtocolError("invalid_request", "payload must be an object", op, request_id)
    fields: dict[str, Any] = dict(payload or {})
    reserved_fields = {"op", "payload", *REQUEST_ID_ALIASES}
    fields.update({key: item for key, item in value.items() if key not in reserved_fields})
    return OperatorRequest(op=op, request_id=request_id, fields=fields)


def validate_request(request: OperatorRequest) -> OperatorRequest:
    """Validate the protocol surface that is independent from ROS services."""

    if request.op in FIXED_DATUM_MUTATIONS:
        raise ProtocolError(
            "UNSUPPORTED_FIXED_DATUM",
            "datum mutation is unavailable while SALUS uses a fixed datum",
            request.op,
            request.request_id,
        )
    if request.op not in KNOWN_OPERATIONS:
        raise ProtocolError(
            "unknown_op", "unsupported operation", request.op, request.request_id
        )

    fields = dict(request.fields)
    if request.op in {"set_control_lock", "set_manual_mode"}:
        _require_bool(fields, "enabled", request)
    elif request.op == "set_manual_cmd":
        fields["linear_x"] = _finite_number(fields, "linear_x", request)
        fields["angular_z"] = _finite_number(fields, "angular_z", request)
        brake_pct = _finite_number(fields, "brake_pct", request, default=0.0)
        fields["brake_pct"] = max(0.0, min(100.0, brake_pct))
    elif request.op == "set_navigation_profile":
        profile = fields.get("profile")
        if profile not in {"urban", "rural"}:
            raise ProtocolError(
                "invalid_request", "profile must be urban or rural", request.op, request.request_id
            )
    elif request.op == "set_sensor_info_view":
        _require_bool(fields, "enabled", request)
        interval = fields.get("interval_s")
        if interval is not None:
            fields["interval_s"] = _finite_number(fields, "interval_s", request)
        tab = fields.get("tab")
        if tab is not None and not isinstance(tab, str):
            raise ProtocolError(
                "invalid_request", "tab must be a string", request.op, request.request_id
            )
    return OperatorRequest(request.op, request.request_id, fields)


def ack(
    request: OperatorRequest | ProtocolError | str,
    *,
    ok: bool,
    error: str | None = None,
    error_code: str | None = None,
    **fields: Any,
) -> dict[str, Any]:
    """Build the legacy-compatible common acknowledgement shape."""

    if isinstance(request, OperatorRequest):
        request_op = request.op
        request_id = request.request_id
    elif isinstance(request, ProtocolError):
        request_op = request.request
        request_id = request.request_id
    else:
        request_op = request
        request_id = None
    result: dict[str, Any] = {"op": "ack", "request": request_op, "ok": ok, "error": error}
    if request_id is not None:
        result["client_req_id"] = request_id
    if error_code is not None:
        result["error_code"] = error_code
    result.update(fields)
    return result


def is_controlled_operation(request: OperatorRequest) -> bool:
    """Return whether the web lock must authorize this particular request."""

    if request.op in {
        "set_goal_ll", "set_route_ll", "set_patrol_ll", "set_navigation_profile",
        "request_return_home", "set_manual_cmd",
    }:
        return True
    return request.op == "set_manual_mode" and request.fields.get("enabled") is True


def _request_id(value: Mapping[str, Any], op: str) -> str | None:
    supplied = [
        value[key]
        for key in REQUEST_ID_ALIASES
        if key in value and value[key] is not None
    ]
    if not supplied:
        return None
    if any(not isinstance(item, (str, int)) for item in supplied):
        raise ProtocolError("invalid_request", "request id must be a string or integer", op)
    normalized = [str(item) for item in supplied]
    if len(set(normalized)) != 1:
        raise ProtocolError("invalid_request", "request id aliases disagree", op)
    return normalized[0]


def _finite_number(
    fields: Mapping[str, Any],
    key: str,
    request: OperatorRequest,
    *,
    default: float | None = None,
) -> float:
    value = fields.get(key, default)
    if isinstance(value, bool):
        value = None
    try:
        number = float(value)
    except (TypeError, ValueError):
        number = math.nan
    if not math.isfinite(number):
        raise ProtocolError(
            "invalid_request", f"{key} must be finite", request.op, request.request_id
        )
    return number


def _require_bool(fields: Mapping[str, Any], key: str, request: OperatorRequest) -> None:
    if not isinstance(fields.get(key), bool):
        raise ProtocolError(
            "invalid_request", f"{key} must be boolean", request.op, request.request_id
        )
