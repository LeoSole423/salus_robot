from pathlib import Path

import pytest

from salus_web.waypoint_repository import (
    AtomicWaypointRepository,
    WaypointValidationError,
    normalize_document,
    parse_yaml,
)


def _document() -> dict:
    return {
        "waypoints": [
            {"latitude": -31.2, "longitude": -64.2, "yaw": 30.0, "role": "home"},
            {"lat": -31.3, "lon": -64.3, "actions": [{"type": "brake_hold", "duration_s": 1.0}]},
        ],
        "patrol_profile": {
            "home_waypoint_index": 0,
            "loop_waypoint_indices": [1],
            "return_waypoint_indices": [],
            "depart_waypoint_indices": [],
            "depart_entry_waypoint_index": 1,
        },
    }


def test_repository_round_trip_is_canonical_and_atomic(tmp_path: Path) -> None:
    repository = AtomicWaypointRepository(tmp_path / "routes" / "waypoints.yaml")
    expected = normalize_document(_document())
    repository.save(expected)
    assert repository.load() == expected
    assert "latitude" in repository.path.read_text(encoding="utf-8")
    assert not list(repository.path.parent.glob(".*.tmp"))


def test_invalid_save_does_not_replace_existing_document(tmp_path: Path) -> None:
    repository = AtomicWaypointRepository(tmp_path / "waypoints.yaml")
    repository.save(normalize_document(_document()))
    before = repository.path.read_bytes()
    forged = normalize_document(_document())
    forged.waypoints[0]["lat"] = float("nan")
    with pytest.raises(WaypointValidationError):
        repository.save(forged)
    assert repository.path.read_bytes() == before


def test_parser_rejects_invalid_coordinate_and_multiple_home() -> None:
    with pytest.raises(WaypointValidationError, match="out of range"):
        parse_yaml("waypoints:\n  - latitude: 100\n    longitude: 0\n")
    with pytest.raises(WaypointValidationError, match="only one HOME"):
        normalize_document({
            "waypoints": [
                {"lat": 0, "lon": 0, "role": "home"},
                {"lat": 1, "lon": 1, "role": "home"},
            ]
        })
