"""Keep the public zone contracts and Nav2 keepout wiring explicit."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
REPO = ROOT.parents[0]


def test_zone_contracts_and_launch_are_installed():
    cmake = (REPO / "salus_interfaces" / "CMakeLists.txt").read_text(encoding="utf-8")
    launch = (ROOT / "launch" / "navigation_zones_sim.launch.py").read_text(encoding="utf-8")
    assert '"srv/SetZonesGeoJson.srv"' in cmake
    assert '"srv/GetZonesState.srv"' in cmake
    assert '"msg/ProjectedKeepoutPolygon.msg"' in cmake
    assert '"msg/ProjectedKeepoutState.msg"' in cmake
    assert "keepout_filter_mask_server" in launch
    assert "costmap_filter_info_server" in launch
    assert "lifecycle_manager_keepout_filters" in launch
    assert '"bond_timeout": 15.0' in launch
    assert '"service_timeout_s": 15.0' in launch
    assert '"initial_reload_max_attempts": 20' in launch
    assert "TimerAction" not in launch


def test_projected_keepout_state_is_transient_local_and_reliable():
    manager = (ROOT / "salus_navigation" / "zones_manager.py").read_text(
        encoding="utf-8"
    )
    assert '"/zones_manager/projected_keepouts"' in manager
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in manager
    assert "ReliabilityPolicy.RELIABLE" in manager
    assert "depth=1" in manager
    accept = manager[
        manager.index("    def _accept_active_state("):
        manager.index("    def _reload_map(")
    ]
    assert accept.index("self._projected_revision += 1") < accept.index(
        "self._projected_pub.publish(message)"
    )


def test_costmaps_and_runtime_data_are_separated():
    config = (ROOT / "config" / "nav2_core_sim.yaml").read_text(encoding="utf-8")
    manager = (ROOT / "salus_navigation" / "zones_manager.py").read_text(encoding="utf-8")
    assert config.count("keepout_filter") >= 4
    assert "runtime/zones" in manager
    assert "map_server_load_map+global_costmap_clear" in manager
    assert "callback_group=self._service_group" in manager


def test_zone_manager_waits_for_active_map_server_before_generating_or_loading_mask():
    manager = (ROOT / "salus_navigation" / "zones_manager.py").read_text(
        encoding="utf-8"
    )
    assert "from lifecycle_msgs.srv import GetState" in manager
    assert '"/keepout_filter_mask_server/get_state"' in manager
    assert "keepout map server not active" in manager

    initial = manager[
        manager.index("    def _load_initial_state"):
        manager.index("    def _await")
    ]
    assert initial.index("self._require_map_server_active()") < initial.index(
        "self._apply("
    )

    apply = manager[
        manager.index("    def _apply("):
        manager.index("    def _set_geojson")
    ]
    assert apply.index("self._require_map_server_active()") < apply.index(
        "self._write_mask(document)"
    )
