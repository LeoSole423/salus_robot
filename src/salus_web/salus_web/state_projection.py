"""Pure projections from cached operator data to Cockpit payloads."""

from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping

from .operator_guard import OperatorLockState


def project_state(
    cached: Mapping[str, Any],
    control: OperatorLockState,
    *,
    request_id: str | None = None,
) -> dict[str, Any]:
    """Build a coherent full-state payload without reading ROS or disk."""

    payload = _copy_mapping(cached)
    payload.update(
        {
            "op": "state",
            "ok": True,
            "control_locked": control.locked,
            "control_lock_reason": control.reason,
            "locked": control.locked,
            "lock_reason": control.reason,
        }
    )
    if request_id is not None:
        payload["client_req_id"] = request_id
    return payload


def project_nav_telemetry(cached: Mapping[str, Any], control: OperatorLockState) -> dict[str, Any]:
    """Build the replaceable compact telemetry broadcast."""

    payload = _copy_mapping(cached)
    payload.update(
        {
            "op": "nav_telemetry",
            "control_locked": control.locked,
            "control_lock_reason": control.reason,
        }
    )
    return payload


def unavailable_sensor_info(tab: str) -> dict[str, Any]:
    """Describe a deferred sensor view without pretending data is available."""

    return {"op": "sensor_info", "tab": tab, "implemented": False}


def _copy_mapping(value: Mapping[str, Any]) -> dict[str, Any]:
    return deepcopy(dict(value))
