from copy import deepcopy
from pathlib import Path

import yaml


ROOT = Path(__file__).parents[2]
BRINGUP = ROOT / "salus_bringup"
NAVIGATION = ROOT / "salus_navigation"


def _config(name: str):
    return yaml.safe_load(
        (NAVIGATION / "config" / name).read_text(encoding="utf-8")
    )


def test_no_keepout_diagnostic_profile_changes_only_keepout_filters() -> None:
    base = _config("nav2_core_no_obstacles_sim.yaml")
    diagnostic = _config("nav2_core_no_obstacles_no_keepout_diag.yaml")
    expected = deepcopy(base)

    for costmap_name in ("local_costmap", "global_costmap"):
        params = expected[costmap_name][costmap_name]["ros__parameters"]
        assert params.pop("filters") == ["keepout_filter"]
        keepout = params.pop("keepout_filter")
        assert keepout["plugin"] == "nav2_costmap_2d::KeepoutFilter"
        assert keepout["enabled"] is True

    assert diagnostic == expected


def test_integration_launch_exposes_safe_no_zones_control_without_changing_defaults() -> None:
    source = (BRINGUP / "launch" / "integration_sim.launch.py").read_text(
        encoding="utf-8"
    )

    assert '"launch_zones"' in source
    assert '"nav2_no_obstacles_params_file"' in source
    assert "nav2_core_no_obstacles_no_keepout_diag.yaml" not in source
    assert "launch_zones == \"false\" and use_keepout == \"true\"" in source
    assert "use_keepout=true requires launch_zones=true" in source

    zones_section = source.split('"navigation_zones_sim.launch.py"', 1)[1].split(
        '"navigation_core_sim.launch.py"', 1
    )[0]
    assert "launch_navigation" in zones_section
    assert "launch_zones" in zones_section

    default_section = source.split(
        'DeclareLaunchArgument(\n                "launch_zones"', 1
    )[1].split("),", 1)[0]
    assert 'default_value="true"' in default_section
