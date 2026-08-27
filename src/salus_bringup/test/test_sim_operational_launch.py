from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "sim_operational.launch.py"
INTEGRATION = Path(__file__).parents[1] / "launch" / "integration_sim.launch.py"


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
