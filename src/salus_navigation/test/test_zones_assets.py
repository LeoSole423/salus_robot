"""Keep the public zone contracts and Nav2 keepout wiring explicit."""

from pathlib import Path


ROOT = Path(__file__).parents[1]
REPO = ROOT.parents[0]


def test_zone_contracts_and_launch_are_installed():
    cmake = (REPO / "salus_interfaces" / "CMakeLists.txt").read_text(encoding="utf-8")
    launch = (ROOT / "launch" / "navigation_zones_sim.launch.py").read_text(encoding="utf-8")
    assert '"srv/SetZonesGeoJson.srv"' in cmake
    assert '"srv/GetZonesState.srv"' in cmake
    assert "keepout_filter_mask_server" in launch
    assert "costmap_filter_info_server" in launch
    assert "lifecycle_manager_keepout_filters" in launch
    assert '"bond_timeout": 15.0' in launch


def test_costmaps_and_runtime_data_are_separated():
    config = (ROOT / "config" / "nav2_core_sim.yaml").read_text(encoding="utf-8")
    manager = (ROOT / "salus_navigation" / "zones_manager.py").read_text(encoding="utf-8")
    assert config.count("keepout_filter") >= 4
    assert "runtime/zones" in manager
    assert "map_server_load_map+global_costmap_clear" in manager
