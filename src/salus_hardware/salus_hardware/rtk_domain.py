"""Pure RTCM validation and GNSS/RTK status policies."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from enum import IntEnum
from typing import Any, Mapping


class FixQuality(IntEnum):
    UNKNOWN = 0
    NO_FIX = 1
    AUTONOMOUS = 2
    DGPS = 3
    RTK_FLOAT = 4
    RTK_FIXED = 5


class AcquisitionState(IntEnum):
    DISABLED = 0
    DISCONNECTED = 1
    CONNECTED_NO_DATA = 2
    RECEIVING = 3
    STALE = 4
    ERROR = 5


class DeliveryBackend(IntEnum):
    DISABLED = 0
    PIXHAWK_MAVROS = 1
    DIRECT_USB = 2


class DeliveryState(IntEnum):
    DISABLED = 0
    IDLE = 1
    DELIVERING = 2
    STALE = 3
    ERROR = 4


@dataclass(frozen=True)
class LegacySourceStatus:
    sequence: int
    source_id: str
    connected: bool
    receiving: bool
    correction_age_s: float
    received_count: int
    crc_error_count: int
    detail: str


def crc24q(data: bytes) -> int:
    """Return the RTCM CRC-24Q of *data*."""

    crc = 0
    for octet in data:
        crc ^= octet << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF


def validate_rtcm3_frame(data: bytes) -> str:
    """Return ``accepted`` or a stable rejection reason for one RTCM3 frame."""

    if not data:
        return "empty"
    if len(data) < 6:
        return "too_short"
    if len(data) > 1029:
        return "too_large"
    if data[0] != 0xD3:
        return "invalid_preamble"
    if data[1] & 0xFC:
        return "reserved_bits_set"
    payload_size = ((data[1] & 0x03) << 8) | data[2]
    if len(data) != payload_size + 6:
        return "length_mismatch"
    expected_crc = int.from_bytes(data[-3:], byteorder="big")
    if crc24q(data[:-3]) != expected_crc:
        return "crc_mismatch"
    return "accepted"


def map_receiver_fix_type(fix_type: int | None) -> FixQuality:
    """Map MAVLink GPS_RAW_INT fix_type without consulting RTCM transport."""

    if fix_type is None:
        return FixQuality.UNKNOWN
    if fix_type <= 1:
        return FixQuality.NO_FIX
    if fix_type in (2, 3, 7, 8):
        return FixQuality.AUTONOMOUS
    if fix_type == 4:
        return FixQuality.DGPS
    if fix_type == 5:
        return FixQuality.RTK_FLOAT
    if fix_type == 6:
        return FixQuality.RTK_FIXED
    return FixQuality.UNKNOWN


def map_legacy_fix_status(value: str) -> tuple[FixQuality, int]:
    """Best-effort compatibility map for the legacy textual status."""

    normalized = value.strip().lower()
    mapping = {
        "gps_no_fix": (FixQuality.NO_FIX, 1),
        "rtk_float": (FixQuality.RTK_FLOAT, 5),
        "rtk_fixed": (FixQuality.RTK_FIXED, 6),
        "gps_only": (FixQuality.AUTONOMOUS, 3),
        "gps_static": (FixQuality.AUTONOMOUS, 7),
        "ppp": (FixQuality.AUTONOMOUS, 8),
    }
    # ``rtk_fix`` is intentionally unknown: the legacy fallback can emit it
    # without an unambiguous Float/Fixed receiver fix type.
    return mapping.get(normalized, (FixQuality.UNKNOWN, -1))


def delivery_backend(value: str) -> DeliveryBackend:
    normalized = value.strip().lower()
    mapping = {
        "disabled": DeliveryBackend.DISABLED,
        "pixhawk_mavros": DeliveryBackend.PIXHAWK_MAVROS,
        "direct_usb": DeliveryBackend.DIRECT_USB,
    }
    if normalized not in mapping:
        raise ValueError("delivery_backend must be disabled, pixhawk_mavros or direct_usb")
    return mapping[normalized]


def parse_legacy_source_status(raw: str, *, stale_timeout_s: float) -> LegacySourceStatus:
    """Parse and sanitize legacy JSON without propagating paths or credentials."""

    if not math.isfinite(stale_timeout_s) or stale_timeout_s <= 0.0:
        raise ValueError("stale_timeout_s must be positive and finite")
    try:
        payload: Mapping[str, Any] = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError("invalid_status_json") from error
    if not isinstance(payload, Mapping):
        raise ValueError("invalid_status_json")

    age_value = payload.get("rtcm_age_s")
    age = float(age_value) if age_value is not None else math.nan
    if math.isfinite(age):
        age = max(0.0, age)
    connected = bool(payload.get("connected", False))
    receiving = bool(payload.get("receiving_rtcm", False))
    error = str(payload.get("last_error") or "").strip()
    if error:
        detail = "legacy_source_error"
    elif receiving:
        detail = "receiving_rtcm"
    elif connected:
        detail = "connected_without_fresh_rtcm"
    else:
        detail = "disconnected"
    return LegacySourceStatus(
        sequence=max(0, int(payload.get("status_sequence", 0))),
        source_id=str(payload.get("active_source_id") or "").strip(),
        connected=connected,
        receiving=receiving,
        correction_age_s=age,
        received_count=max(0, int(payload.get("received_count", 0))),
        crc_error_count=max(0, int(payload.get("crc_errors", 0))),
        detail=detail,
    )


def acquisition_state(
    status: LegacySourceStatus | None, *, stale_timeout_s: float
) -> AcquisitionState:
    if status is None:
        return AcquisitionState.DISCONNECTED
    if status.detail == "legacy_source_error":
        return AcquisitionState.ERROR
    if not status.connected:
        return AcquisitionState.DISCONNECTED
    if status.received_count == 0:
        return AcquisitionState.CONNECTED_NO_DATA
    if not math.isfinite(status.correction_age_s):
        return AcquisitionState.CONNECTED_NO_DATA
    if status.correction_age_s > stale_timeout_s or not status.receiving:
        return AcquisitionState.STALE
    return AcquisitionState.RECEIVING


def age_source_status(
    status: LegacySourceStatus | None, *, elapsed_s: float
) -> LegacySourceStatus | None:
    """Advance a reported correction age using an explicit steady duration."""

    if status is None or not math.isfinite(status.correction_age_s):
        return status
    if not math.isfinite(elapsed_s) or elapsed_s < 0.0:
        raise ValueError("elapsed_s must be non-negative and finite")
    return replace(status, correction_age_s=status.correction_age_s + elapsed_s)


def sequence_transition(previous: int | None, current: int) -> str:
    """Classify status sequence changes; regressions never imply fresh data."""

    if current < 0:
        return "invalid"
    if previous is None:
        return "first"
    if current > previous:
        return "advanced"
    if current == previous:
        return "duplicate"
    return "reset"
