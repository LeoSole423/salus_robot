from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_snapshot_server_keeps_adr_boundaries_and_contract_topics() -> None:
    source = (ROOT / "salus_navigation/nav_snapshot_server.py").read_text()
    launch = (ROOT / "launch/navigation_snapshot_sim.launch.py").read_text()
    assert '"/nav_snapshot_server/get_nav_snapshot"' in source
    assert '"/scan_clean"' in source
    assert '"/stop_zone_raw"' in source
    assert '"/scan_preview"' not in source
    assert "MISSING_LOCAL_COSTMAP" in source
    assert "STALE_LOCAL_COSTMAP" in source
    assert "MISSING_LOCAL_TF" in source
    assert "nav_snapshot_server" in launch
