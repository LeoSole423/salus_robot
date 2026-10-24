from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
from pathlib import Path
from unittest.mock import patch

import pytest

from salus_web.route_repository import RouteError, RouteRepository


def document():
    return {"waypoints": [{"localId": "home", "x": -31., "y": -64., "role": "home"},
                          {"localId": "loop", "x": -31.1, "y": -64.1, "yawDeg": 45.,
                           "actions": [{"type": "brake_hold", "duration_s": 4},
                                       {"type": "set_navigation_profile", "profile": "rural"}]}],
            "loopRoute": False,
            "patrolMissionProfileRefs": {"homeWaypointIndex": 0, "loopWaypointIndices": [1],
                "departWaypointIndices": [], "returnWaypointIndices": [], "departEntryWaypointIndex": 1}}


def save(repo, **kwargs):
    return repo.dispatch("save_route", {"name": "Ronda", "document": document(), **kwargs})["route"]


def test_restart_and_full_round_trip(tmp_path):
    path = tmp_path / "web" / "routes.json"
    route = save(RouteRepository(path))
    reopened = RouteRepository(path)
    assert reopened.dispatch("get_route", {"id": route["id"]})["route"]["document"] == document()
    assert reopened.dispatch("list_routes", {})["routes"] == [
        {k: route[k] for k in ("id", "name", "revision")}]
    assert not list(path.parent.glob("*.tmp"))


def test_duplicate_create_and_stale_update_delete(tmp_path):
    repo = RouteRepository(tmp_path / "routes.json")
    first = save(repo)
    with pytest.raises(RouteError, match="nombre"):
        save(repo)
    updated = save(repo, id=first["id"], expected_revision=1)
    assert updated["revision"] == 2
    for op in ("save_route", "delete_route"):
        with pytest.raises(RouteError) as error:
            repo.dispatch(op, {"id": first["id"], "expected_revision": 1,
                               "name": "Ronda", "document": document()})
        assert error.value.code == "ROUTE_CONFLICT"
    repo.dispatch("delete_route", {"id": first["id"], "expected_revision": 2})
    assert repo.dispatch("list_routes", {})["routes"] == []
    assert save(repo)["id"] != first["id"]


def test_two_writers_cannot_overwrite_same_revision(tmp_path):
    path = tmp_path / "routes.json"
    first = save(RouteRepository(path))
    def update(_):
        try:
            return save(RouteRepository(path), id=first["id"], expected_revision=1)["revision"]
        except RouteError as exc:
            return exc.code
    with ThreadPoolExecutor(2) as pool:
        results = list(pool.map(update, range(2)))
    assert sorted(map(str, results)) == ["2", "ROUTE_CONFLICT"]


@pytest.mark.parametrize("change", [
    lambda d: d["waypoints"][0].update(x=float("nan")),
    lambda d: d.update(waypoints=[]),
    lambda d: d.update(loopRoute="false"),
    lambda d: d["patrolMissionProfileRefs"].update(loopWaypointIndices=[9]),
    lambda d: d["waypoints"][1].update(actions=[{"type": "unknown"}]),
])
def test_invalid_definition_never_overwrites(tmp_path, change):
    repo = RouteRepository(tmp_path / "routes.json")
    first = save(repo)
    before = repo.path.read_bytes()
    bad = deepcopy(document()); change(bad)
    with pytest.raises(RouteError):
        save(repo, id=first["id"], expected_revision=1, document=bad)
    assert repo.path.read_bytes() == before


def test_disk_failure_keeps_previous_file(tmp_path):
    repo = RouteRepository(tmp_path / "routes.json")
    first = save(repo)
    before = repo.path.read_bytes()
    with patch("salus_web.route_repository.os.replace", side_effect=OSError("disk full")):
        with pytest.raises(OSError):
            save(repo, id=first["id"], expected_revision=1)
    assert repo.path.read_bytes() == before
    assert not list(tmp_path.glob("*.tmp"))


def test_corrupt_store_is_not_replaced(tmp_path):
    path = tmp_path / "routes.json"
    path.write_text('{"schema_version":999,"routes":[]}')
    with pytest.raises(RouteError):
        save(RouteRepository(path))
    assert json.loads(path.read_text())["schema_version"] == 999
