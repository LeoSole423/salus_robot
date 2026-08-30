from pathlib import Path


ROOT = Path(__file__).parents[1]
REPO_ROOT = Path(__file__).parents[3]


def test_snapshot_server_keeps_adr_boundaries_and_contract_topics() -> None:
    source = (ROOT / "salus_navigation/nav_snapshot_server.py").read_text()
    launch = (ROOT / "launch/navigation_snapshot_sim.launch.py").read_text()
    config = (ROOT / "config/navigation_snapshot.yaml").read_text()
    assert '"/nav_snapshot_server/get_nav_snapshot"' in source
    assert '"/scan_clean"' in source
    assert '"/stop_zone_raw"' in source
    assert '"/scan_preview"' not in source
    assert "MISSING_LOCAL_COSTMAP" in source
    assert "STALE_LOCAL_COSTMAP" in source
    assert "MISSING_LOCAL_TF" in source
    assert "MultiThreadedExecutor(num_threads=2)" in source
    assert "callback_group=self._cache_callbacks" in source
    assert "callback_group=self._service_callbacks" in source
    assert "nav_snapshot_server" in launch
    assert "navigation_snapshot.yaml" in launch
    assert "local_costmap_max_age_s: 2.0" in config
    assert "dynamic_layer_max_age_s: 2.0" in config


def test_snapshot_smoke_waits_for_causal_navigation_startup_before_polling() -> None:
    source = (REPO_ROOT / "tools/smoke_navigation_snapshot.py").read_text(
        encoding="utf-8"
    )
    startup_wait = source.index('"navigation startup"')
    poller = source.index("poller = AsyncServicePoller")
    snapshot_wait = source.index('"snapshot readiness"')
    assert "subscribe_navigation_startup" in source
    assert "runtime.wait_navigation_startup" in source
    assert "node.startup, 60.0" in source
    assert startup_wait < poller < snapshot_wait
    assert '"navigation_startup": node.startup.snapshot()' in source
    assert 'create_publisher(' in source
    assert 'LaserScan, "/scan_clean"' in source
    assert 'message.header.stamp = self.local_odom.header.stamp' in source
    assert "fixture_scan_interval_s = 0.5" in source
    assert "now < self.next_fixture_scan_at" in source
    assert "response_timeout_s=20.0" in source

    wrapper = (REPO_ROOT / "tools/smoke_navigation_snapshot.sh").read_text(
        encoding="utf-8"
    )
    assert "capability_profile:=no_obstacle_detection" in wrapper
