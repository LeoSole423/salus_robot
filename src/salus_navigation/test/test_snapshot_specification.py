import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
FIXTURE = Path(__file__).parent / "fixtures/navigation_snapshots/scenarios.json"


def test_snapshot_contracts_preserve_legacy_fields():
    msg = (ROOT / "src/salus_interfaces/msg/NavSnapshotLayers.msg").read_text()
    srv = (ROOT / "src/salus_interfaces/srv/GetNavSnapshot.srv").read_text()
    assert msg.splitlines() == [
        "bool local_costmap",
        "bool global_costmap",
        "bool keepout_mask",
        "bool footprint",
        "bool stop_zone",
        "bool scan",
        "bool plan",
        "bool collision_polygons",
        "bool global_inset",
    ]
    assert srv.splitlines() == [
        "---",
        "bool ok",
        "string error",
        "string mime",
        "uint32 width",
        "uint32 height",
        "string frame_id",
        "builtin_interfaces/Time stamp",
        "salus_interfaces/NavSnapshotLayers layers",
        "uint8[] image_png",
    ]


def test_snapshot_fixture_is_complete_and_decision_locked():
    data = json.loads(FIXTURE.read_text())
    assert data["schema_version"] == 1
    assert data["render"] == {
        "extent_m": 30.0,
        "size_px": 512,
        "global_inset_px": 160,
        "frame_id": "odom",
        "base_frame": "base_footprint",
        "colors_bgr": data["render"]["colors_bgr"],
    }
    scenarios = {item["id"]: item for item in data["scenarios"]}
    assert set(scenarios) == {
        "local_required_only",
        "all_layers_transform_and_clip",
        "optional_layers_missing_or_stale",
        "missing_local_costmap",
        "stale_local_costmap",
        "missing_local_tf",
    }
    assert scenarios["all_layers_transform_and_clip"]["expected"][
        "forbidden_decorations"
    ] == ["synthetic_vehicle", "legend", "direction_markers"]
    for scenario_id in ("missing_local_costmap", "stale_local_costmap", "missing_local_tf"):
        expected = scenarios[scenario_id]["expected"]
        assert expected["ok"] is False
        assert expected["image_bytes"] == 0
