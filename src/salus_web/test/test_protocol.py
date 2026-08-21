import json
from pathlib import Path

import pytest

from salus_web.protocol import ProtocolError, ack, parse_request, validate_request


FIXTURE = Path(__file__).parent / "fixtures/cockpit_protocol/scenarios.json"


def test_fixture_correlation_alias_is_normalized() -> None:
    scenario = next(
        item
        for item in json.loads(FIXTURE.read_text())["cases"]
        if item["id"] == "correlate_alias"
    )
    request = validate_request(parse_request(scenario["request"]))
    assert request.op == "get_state"
    assert request.request_id == "req-1"
    assert ack(request, ok=True) == {
        "op": "ack",
        "request": "get_state",
        "ok": True,
        "error": None,
        "client_req_id": "req-1",
    }


def test_payload_fields_are_accepted_but_top_level_wins() -> None:
    request = validate_request(parse_request({
        "op": "set_manual_cmd",
        "request_id": "a",
        "payload": {"linear_x": 1.0, "angular_z": 0.2, "brake_pct": 30},
        "linear_x": 2.0,
    }))
    assert request.fields == {"linear_x": 2.0, "angular_z": 0.2, "brake_pct": 30.0}


def test_invalid_json_and_unknown_operation_have_bounded_errors() -> None:
    with pytest.raises(ProtocolError) as invalid:
        parse_request("{")
    assert invalid.value.code == "invalid_json"
    response = ack(
        invalid.value.request,
        ok=False,
        error=invalid.value.message,
        error_code=invalid.value.code,
    )
    assert response["request"] == "invalid_json"

    with pytest.raises(ProtocolError) as unknown:
        validate_request(
            parse_request({"op": "not_supported", "client_req_id": "req-2"})
        )
    assert unknown.value.code == "unknown_op"
    assert unknown.value.request_id == "req-2"


def test_fixed_datum_mutation_is_explicitly_rejected() -> None:
    with pytest.raises(ProtocolError) as rejected:
        validate_request(
            parse_request({"op": "select_datum", "client_req_id": "req-7", "id": "old-profile"})
        )
    assert rejected.value.code == "UNSUPPORTED_FIXED_DATUM"
    response = ack(
        rejected.value.request,
        ok=False,
        error=rejected.value.message,
        error_code=rejected.value.code,
    )
    assert response["error_code"] == "UNSUPPORTED_FIXED_DATUM"


def test_manual_command_keeps_top_level_contract_and_clamps_brake() -> None:
    request = validate_request(parse_request({
        "op": "set_manual_cmd",
        "client_req_id": "req-5",
        "linear_x": 1.0,
        "angular_z": 0.2,
        "brake_pct": 150,
    }))
    assert request.fields == {"linear_x": 1.0, "angular_z": 0.2, "brake_pct": 100.0}


def test_conflicting_request_id_aliases_are_rejected() -> None:
    with pytest.raises(ProtocolError, match="aliases disagree"):
        parse_request({"op": "get_state", "requestId": "one", "client_req_id": "two"})
