import json

from salus_navigation.patrol_domain import PatrolMissionSpec, PatrolRoute
import pytest

from salus_navigation.patrol_store import decode, encode, write_atomic
from salus_navigation.route_model import RouteWaypoint


def spec():
    point = RouteWaypoint(-31.0, -64.0, 0.0, 0, map_x=1.0, map_y=2.0)
    route = PatrolRoute((point, point), ("", ""))
    return PatrolMissionSpec(point, route, PatrolRoute((), ()), PatrolRoute((), ()), 0, 2.0, 20.0, 5)


def test_atomic_store_writes_versioned_document(tmp_path):
    target = tmp_path / "runtime" / "missions" / "patrol.json"
    write_atomic(target, spec())
    payload = json.loads(target.read_text(encoding="utf-8"))
    assert payload == encode(spec())
    assert decode(payload) == spec()
    assert not list(target.parent.glob("tmp*"))


def test_store_preserves_yaw_provenance_and_defaults_missing_legacy_metadata_to_automatic():
    payload = encode(spec())
    payload["home"]["yaw_explicit"] = True
    payload["loop"]["waypoints"][0]["yaw_explicit"] = True
    payload["loop"]["waypoints"][1].pop("yaw_explicit")

    decoded = decode(payload)

    assert decoded.home.yaw_explicit
    assert decoded.loop.waypoints[0].yaw_explicit
    assert not decoded.loop.waypoints[1].yaw_explicit


def test_decode_rejects_unknown_schema_and_malformed_route():
    payload = encode(spec())
    payload["schema_version"] = 99
    with pytest.raises(ValueError, match="schema"):
        decode(payload)
    payload = encode(spec())
    payload["loop"]["actions"] = [3]
    with pytest.raises(ValueError, match="actions"):
        decode(payload)
