"""Pure contract tests for the private georeferenced patrol preparation tool."""

import importlib.util
import json
from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).parents[3]
TOOL = ROOT / "tools" / "prepare_georeferenced_patrol_sim.py"
SPEC = importlib.util.spec_from_file_location("prepare_georeferenced_patrol_sim", TOOL)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _route() -> dict:
    return {
        "PatrullaSencillaPolo": {
            "waypoints": [
                {"localId": "home", "x": -31.5, "y": -64.2, "role": "home"},
                {"localId": "depart", "x": -31.5, "y": -64.1999},
                {"localId": "loop", "x": -31.4999, "y": -64.1999, "yawDeg": 90.0},
            ],
            "patrolMissionProfileRefs": {
                "homeWaypointIndex": 0,
                "loopWaypointIndices": [2],
                "returnWaypointIndices": [],
                "departWaypointIndices": [1],
                "departEntryWaypointIndex": 2,
            },
        }
    }


def _world(path: Path) -> None:
    path.write_text(
        "<sdf version=\"1.6\"><world name=\"test\">"
        "<spherical_coordinates><latitude_deg>0</latitude_deg>"
        "<longitude_deg>0</longitude_deg></spherical_coordinates>"
        "</world></sdf>",
        encoding="utf-8",
    )


def test_preparer_keeps_route_private_and_writes_relative_fixture(tmp_path, monkeypatch) -> None:
    monkeypatch.chdir(tmp_path)
    artifacts = tmp_path / "artifacts"
    artifacts.mkdir()
    routes = artifacts / "routes-private.json"
    routes.write_text(json.dumps(_route()), encoding="utf-8")
    world = artifacts / "free.world"
    _world(world)

    result = MODULE.prepare_artifacts(
        routes_file=routes,
        route_name="PatrullaSencillaPolo",
        source_world=world,
        output_dir=artifacts / "run",
    )

    sanitized = yaml.safe_load(result["sanitized_path"].read_text(encoding="utf-8"))
    assert sanitized["home_input_index"] == 0
    assert sanitized["spawn"]["x_m"] == 0.0
    assert sanitized["spawn"]["y_m"] == 0.0
    assert len(sanitized["waypoints_relative_to_home"]) == 3
    assert "latitude" not in result["sanitized_path"].read_text(encoding="utf-8")
    assert "-31.5" not in result["sanitized_path"].read_text(encoding="utf-8")
    assert oct(result["mission_path"].stat().st_mode & 0o777) == "0o600"
    assert oct(result["world_path"].stat().st_mode & 0o777) == "0o600"
    assert "<heading_deg>0.0000000000</heading_deg>" in result["world_path"].read_text(
        encoding="utf-8"
    )
    assert "datum_lat:=" in (result["output_dir"] / "launch_args_private.txt").read_text(
        encoding="utf-8"
    )
    assert "rviz:=true" in result["launcher_path"].read_text(encoding="utf-8")
    subprocess.run(["bash", "-n", str(result["launcher_path"])], check=True)


def test_preparer_requires_a_complete_patrol_profile(tmp_path, monkeypatch) -> None:
    monkeypatch.chdir(tmp_path)
    artifacts = tmp_path / "artifacts"
    artifacts.mkdir()
    routes = artifacts / "routes-private.json"
    routes.write_text(json.dumps({"PatrullaSencillaPolo": {"waypoints": []}}), encoding="utf-8")
    world = artifacts / "free.world"
    _world(world)

    with pytest.raises(MODULE.RoutePreparationError):
        MODULE.prepare_artifacts(
            routes_file=routes,
            route_name="PatrullaSencillaPolo",
            source_world=world,
            output_dir=artifacts / "run",
        )
