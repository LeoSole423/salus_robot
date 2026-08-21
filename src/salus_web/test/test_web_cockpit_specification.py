import json
from pathlib import Path


FIXTURE = Path(__file__).parent / "fixtures/cockpit_protocol/scenarios.json"


def test_characterized_protocol_fixture_is_consistent() -> None:
    specification = json.loads(FIXTURE.read_text(encoding="utf-8"))
    assert specification["schema_version"] == 1
    assert specification["transport"]["default_port"] == 8766
    assert len(set(specification["transport"]["request_id_aliases"])) == 4
    required = specification["required_request_ops"]
    assert len(required) == len(set(required))
    assert {"set_goal_ll", "set_route_ll", "set_patrol_ll", "get_nav_snapshot"} <= set(required)
    assert "set_manual_cmd" in required
    assert "state" in specification["required_broadcast_ops"]
    case_ids = [case["id"] for case in specification["cases"]]
    assert len(case_ids) == len(set(case_ids))
    for case in specification["cases"]:
        assert "request" in case or "raw" in case
        assert any(key in case for key in ("expect", "expect_fields", "expect_sequence"))


def test_locked_policy_keeps_stop_operations_available() -> None:
    specification = json.loads(FIXTURE.read_text(encoding="utf-8"))
    controlled = set(specification["controlled_ops"])
    safe = set(specification["safe_while_locked"])
    assert not controlled & safe
    assert {"brake", "cancel_goal", "cancel_route", "cancel_patrol"} <= safe
    assert "set_manual_mode:true" in controlled
    assert "set_manual_mode:false" in safe
