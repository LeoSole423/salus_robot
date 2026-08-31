"""Keep the public zone contracts and vector keepout wiring explicit."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
REPO = ROOT.parents[0]


def test_zone_contracts_and_vector_launch_are_installed():
    cmake = (REPO / "salus_interfaces" / "CMakeLists.txt").read_text(encoding="utf-8")
    launch = (ROOT / "launch" / "navigation_zones_sim.launch.py").read_text(encoding="utf-8")
    assert '"srv/SetZonesGeoJson.srv"' in cmake
    assert '"srv/GetZonesState.srv"' in cmake
    assert "zones_manager" in launch
    assert "keepout_filter_mask_server" not in launch
    assert "costmap_filter_info_server" not in launch
    assert "nav2_lifecycle_manager" not in launch


def test_costmaps_use_bounded_vector_layers_not_legacy_filters():
    config = (ROOT / "config" / "nav2_core_sim.yaml").read_text(encoding="utf-8")
    manager = (ROOT / "salus_navigation" / "zones_manager.py").read_text(encoding="utf-8")
    assert config.count("vector_keepout_layer") >= 4
    assert "KeepoutFilter" not in config
    assert "runtime/zones" in manager
    assert "projected_vector_state" in manager
    assert "LoadMap" not in manager
    assert "keepout_mask.pgm" not in manager
    assert "callback_group=self._service_group" in manager


def test_zone_manager_projects_and_persists_before_committing_vector_state():
    manager = (ROOT / "salus_navigation" / "zones_manager.py").read_text(encoding="utf-8")
    apply = manager[manager.index("    def _apply("):manager.index("    def _set_geojson")]
    assert apply.index("self._project(candidate_document)") < apply.index(
        "self._persist_document(document)"
    ) < apply.index("self._activate_document(")
