import json
import math

import pytest

from salus_hardware.rtk_domain import (
    AcquisitionState,
    DeliveryBackend,
    FixQuality,
    acquisition_state,
    age_source_status,
    crc24q,
    delivery_backend,
    map_legacy_fix_status,
    map_receiver_fix_type,
    parse_legacy_source_status,
    sequence_transition,
    validate_rtcm3_frame,
)


def _frame(payload: bytes = b"\x3e\x00") -> bytes:
    header = bytes((0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF))
    body = header + payload
    return body + crc24q(body).to_bytes(3, byteorder="big")


def _status(**overrides):
    values = {
        "status_sequence": 7,
        "active_source_id": "ign-cordoba",
        "connected": True,
        "receiving_rtcm": True,
        "rtcm_age_s": 0.4,
        "received_count": 20,
        "crc_errors": 2,
        "last_error": None,
        "config_path": "/must/not/escape",
    }
    values.update(overrides)
    return parse_legacy_source_status(json.dumps(values), stale_timeout_s=5.0)


def test_rtcm3_validation_checks_shape_length_and_crc() -> None:
    assert crc24q(b"123456789") == 0xCDE703
    valid = _frame()
    assert validate_rtcm3_frame(valid) == "accepted"
    assert validate_rtcm3_frame(b"") == "empty"
    assert validate_rtcm3_frame(valid[:-1]) == "length_mismatch"
    assert validate_rtcm3_frame(valid[:-1] + bytes((valid[-1] ^ 1,))) == "crc_mismatch"
    assert validate_rtcm3_frame(b"\xd2" + valid[1:]) == "invalid_preamble"
    assert validate_rtcm3_frame(bytes(1030)) == "too_large"


def test_fix_quality_comes_only_from_receiver_fix_type() -> None:
    assert map_receiver_fix_type(1) == FixQuality.NO_FIX
    assert map_receiver_fix_type(3) == FixQuality.AUTONOMOUS
    assert map_receiver_fix_type(4) == FixQuality.DGPS
    assert map_receiver_fix_type(5) == FixQuality.RTK_FLOAT
    assert map_receiver_fix_type(6) == FixQuality.RTK_FIXED
    assert map_receiver_fix_type(None) == FixQuality.UNKNOWN
    assert map_legacy_fix_status("rtcm_ok") == (FixQuality.UNKNOWN, -1)
    assert map_legacy_fix_status("rtk_fix") == (FixQuality.UNKNOWN, -1)


def test_fresh_corrections_and_no_fix_remain_orthogonal() -> None:
    status = _status()
    quality, raw_type = map_legacy_fix_status("gps_no_fix")
    assert acquisition_state(status, stale_timeout_s=5.0) == AcquisitionState.RECEIVING
    assert quality == FixQuality.NO_FIX
    assert raw_type == 1


def test_status_is_sanitized_and_ages_to_stale() -> None:
    status = _status()
    assert status.source_id == "ign-cordoba"
    assert "/must/not/escape" not in repr(status)
    aged = age_source_status(status, elapsed_s=5.0)
    assert aged is not None
    assert aged.correction_age_s == pytest.approx(5.4)
    assert acquisition_state(aged, stale_timeout_s=5.0) == AcquisitionState.STALE
    assert math.isfinite(aged.correction_age_s)


def test_bad_status_and_explicit_backend_are_rejected() -> None:
    with pytest.raises(ValueError, match="invalid_status_json"):
        parse_legacy_source_status("not-json", stale_timeout_s=5.0)
    with pytest.raises(ValueError, match="delivery_backend"):
        delivery_backend("automatic")
    assert delivery_backend("pixhawk_mavros") == DeliveryBackend.PIXHAWK_MAVROS


def test_sequence_regression_is_explicit_and_can_recover() -> None:
    assert sequence_transition(None, 10) == "first"
    assert sequence_transition(10, 11) == "advanced"
    assert sequence_transition(11, 11) == "duplicate"
    assert sequence_transition(11, 1) == "reset"
    assert sequence_transition(1, 2) == "advanced"
