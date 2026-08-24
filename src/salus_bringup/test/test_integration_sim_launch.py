from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "integration_sim.launch.py"


def test_integrated_simulation_composes_all_migrated_subsystems() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    for package in (
        "salus_simulation",
        "salus_control",
        "salus_localization",
        "salus_perception",
        "salus_navigation",
        "salus_web",
    ):
        assert package in contents
    for launch_file in (
        "motion_sim.launch.py",
        "control_sim.launch.py",
        "localization_sim.launch.py",
        "global_localization_sim.launch.py",
        "lidar_sim.launch.py",
        "safety_arbitration_sim.launch.py",
        "navigation_core_sim.launch.py",
        "navigation_zones_sim.launch.py",
        "route_executor_sim.launch.py",
        "patrol_mission_sim.launch.py",
        "navigation_snapshot_sim.launch.py",
        "web_bridge.launch.py",
    ):
        assert launch_file in contents
    assert "launch_navigation" in contents
    assert "use_keepout" in contents
    assert "launch_routes" in contents
    assert "launch_patrol" in contents
    assert "launch_web" in contents
    assert "web_waypoints_file" in contents
    assert "web_telemetry_profile" in contents
    assert "patrol_battery_guard_topic" in contents
    assert contents.count('DeclareLaunchArgument(\n                "world"') == 1


def test_rviz_diagnostics_asset_is_installed_by_perception_package() -> None:
    perception_setup = (
        Path(__file__).parents[2] / "salus_perception" / "setup.py"
    ).read_text(encoding="utf-8")
    diagnostics = (
        Path(__file__).parents[2]
        / "salus_perception"
        / "config"
        / "lidar_diagnostics.rviz"
    ).read_text(encoding="utf-8")
    assert 'glob("config/*.rviz")' in perception_setup
    for topic in (
        "/scan_3d_raw", "/obstacles_cloud", "/scan_clean",
        "/keepout_filter_mask", "/global_costmap/costmap",
        "/local_costmap/costmap",
    ):
        assert topic in diagnostics
