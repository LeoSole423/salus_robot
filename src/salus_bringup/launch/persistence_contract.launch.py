"""Minimal Web + Camera composition for persistence contract testing."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def _include(package: str, launch_file: str, arguments):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description() -> LaunchDescription:
    runtime_dir = LaunchConfiguration("runtime_dir")
    web_ws_port = LaunchConfiguration("web_ws_port")
    return LaunchDescription([
        DeclareLaunchArgument(
            "runtime_dir",
            default_value="runtime/persistence_contract",
            description="Writable root reused across the persistence restart.",
        ),
        DeclareLaunchArgument("web_ws_port", default_value="8766"),
        _include(
            "salus_web",
            "web_bridge.launch.py",
            {
                "ws_port": web_ws_port,
                "waypoints_file": PathJoinSubstitution(
                    [runtime_dir, "web", "waypoints.yaml"]
                ),
                "require_camera_service": "true",
                "scan_preview_enabled": "false",
                "telemetry_profile": "compact",
            },
        ),
        _include(
            "salus_hardware",
            "camera_sim.launch.py",
            {
                "camera_presets_file": PathJoinSubstitution(
                    [runtime_dir, "camera", "presets.json"]
                ),
            },
        ),
    ])
