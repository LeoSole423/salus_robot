"""Contract checks for the known-geometry obstacle-drag measurement world."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml


SIMULATION_DIR = Path(__file__).resolve().parents[1]


def test_obstacle_drag_world_has_three_static_known_boxes() -> None:
    root = ET.parse(SIMULATION_DIR / "worlds" / "obstacle_drag.world").getroot()
    models = {
        model.attrib["name"]: model
        for model in root.findall("./world/model")
        if model.attrib["name"] != "ground_plane"
    }
    assert set(models) == {
        "obstacle_near_post", "obstacle_mid_wall", "obstacle_far_box",
    }
    for model in models.values():
        assert model.findtext("static") == "true"
        assert model.find("./link/collision/geometry/box/size") is not None


def test_world_poses_and_dimensions_match_geometry_fixture() -> None:
    root = ET.parse(SIMULATION_DIR / "worlds" / "obstacle_drag.world").getroot()
    data = yaml.safe_load(
        (SIMULATION_DIR / "config" / "obstacle_drag_geometry.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert data["fixed_frame"] == "odom"
    for obstacle in data["obstacles"]:
        model = root.find(f"./world/model[@name='{obstacle['name']}']")
        assert model is not None
        pose = [float(value) for value in model.findtext("pose").split()]
        assert pose[:2] == pytest.approx([
            obstacle["center_m"]["x"], obstacle["center_m"]["y"]
        ])
        size = [float(value) for value in model.findtext(
            "./link/collision/geometry/box/size"
        ).split()]
        assert size == pytest.approx([
            obstacle["size_m"]["x"], obstacle["size_m"]["y"], obstacle["size_m"]["z"]
        ])


def test_obstacle_drag_smoke_reuses_existing_integration_launch() -> None:
    root = SIMULATION_DIR.parents[1]
    smoke = (root / "tools" / "smoke_obstacle_drag_sim.sh").read_text(
        encoding="utf-8"
    )
    probe = (root / "tools" / "smoke_obstacle_drag_sim.py").read_text(
        encoding="utf-8"
    )
    assert "integration_sim.launch.py" in smoke
    assert "launch_navigation:=false" in smoke
    assert "smoke_harness.sh" in smoke
    assert "/scan_clean" in probe
    assert "--metrics-path" in probe
    assert "obstacle_drag_maneuver.py" in smoke
    maneuver = (root / "tools" / "obstacle_drag_maneuver.py").read_text(
        encoding="utf-8"
    )
    assert 'create_publisher(Twist, "/cmd_vel", 10)' in maneuver
