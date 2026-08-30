from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "sim_operational.launch.py"
INTEGRATION = Path(__file__).parents[1] / "launch" / "integration_sim.launch.py"
PERSISTENCE = Path(__file__).parents[1] / "launch" / "persistence_contract.launch.py"
ROOT = Path(__file__).parents[3]
SIM_OPERATIONAL_SMOKE = ROOT / "tools" / "smoke_sim_operational.sh"
PERSISTENCE_SMOKE = ROOT / "tools" / "smoke_operational_persistence.sh"


def test_operational_profile_wraps_the_checkpoint_without_recomposing_packages() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert "integration_sim.launch.py" in contents
    assert "IncludeLaunchDescription" in contents
    for package in ("salus_control", "salus_navigation", "salus_web"):
        assert package not in contents


def test_operational_profile_has_full_remote_defaults() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    for fragment in (
        '"launch_navigation": "true"',
        '"launch_routes": "true"',
        '"launch_patrol": "true"',
        'DeclareLaunchArgument("launch_web", default_value="true")',
        'DeclareLaunchArgument("launch_camera", default_value="true")',
        'DeclareLaunchArgument("web_ws_port", default_value="8766")',
        'DeclareLaunchArgument("web_telemetry_profile", default_value="compact")',
        '"runtime_dir", default_value="runtime/sim_operational"',
        '"command_input_mode",\n            default_value="legacy_cmd_vel"',
        '"command_input_mode": LaunchConfiguration("command_input_mode")',
        '"capability_profile": LaunchConfiguration("capability_profile")',
        '"imu_source": LaunchConfiguration("imu_source")',
        '"orientation_source": LaunchConfiguration("orientation_source")',
    ):
        assert fragment in contents


def test_operational_profile_partitions_persistent_state() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    for component in ("zones", "patrol", "web", "camera"):
        assert f'runtime_dir, "{component}"' in contents
    integration = INTEGRATION.read_text(encoding="utf-8")
    assert '"runtime_dir": patrol_runtime_dir' in integration


def test_operational_profile_does_not_bind_dds_or_legacy_scan_contracts() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    for forbidden in ("CYCLONEDDS", "ROS_LOCALHOST_ONLY", "scan_wifi_debug"):
        assert forbidden not in contents


def test_operational_smoke_only_owns_composition_contract() -> None:
    contents = SIM_OPERATIONAL_SMOKE.read_text(encoding="utf-8")
    assert "integration_probe.py --operational" in contents
    assert "sim_operational_composition_valid" in contents
    for duplicated in (
        "smoke_route_executor_sim.py",
        "smoke_navigation_profiles.py",
        "smoke_web_cockpit.py",
    ):
        assert duplicated not in contents


def test_persistence_contract_launch_contains_only_state_owners() -> None:
    contents = PERSISTENCE.read_text(encoding="utf-8")
    assert "web_bridge.launch.py" in contents
    assert "camera_sim.launch.py" in contents
    assert '"require_camera_service": "true"' in contents
    assert '"scan_preview_enabled": "false"' in contents
    for unrelated in (
        "integration_sim.launch.py",
        "salus_navigation",
        "salus_simulation",
        "Gazebo",
    ):
        assert unrelated not in contents


def test_persistence_smoke_restarts_minimal_contract_and_not_full_operational() -> None:
    contents = PERSISTENCE_SMOKE.read_text(encoding="utf-8")
    assert contents.count("persistence_contract.launch.py") == 2
    assert "smoke_operational_persistence.py --mode seed" in contents
    assert "smoke_operational_persistence.py --mode verify" in contents
    assert 'smoke_wait "initial cockpit endpoint stopped"' in contents
    assert 'smoke_wait "restarted cockpit endpoint ready"' in contents
    for unrelated in (
        "sim_operational.launch.py",
        "integration_probe.py",
        "smoke_web_cockpit.py",
    ):
        assert unrelated not in contents
