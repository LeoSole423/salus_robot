"""Canonical full-system simulation profile for remote Cockpit operation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression


def generate_launch_description() -> LaunchDescription:
    headless = LaunchConfiguration("headless")
    runtime_dir = LaunchConfiguration("runtime_dir")
    integration = (
        Path(get_package_share_directory("salus_bringup"))
        / "launch"
        / "integration_sim.launch.py"
    )
    gz_args = PythonExpression([
        "'-r -s' if '", headless, "'.lower() == 'true' else '-r'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "vehicle_io_profile", default_value="legacy",
            description="Vehicle measurement/odometry profile: legacy or canonical.",
        ),
        DeclareLaunchArgument(
            "compare_legacy_odometry", default_value="false",
            description="Run legacy odometry on isolated shadow topics.",
        ),
        DeclareLaunchArgument(
            "command_input_mode",
            default_value="legacy_cmd_vel",
            description=(
                "Exclusive control input: legacy_cmd_vel or "
                "canonical_vehicle_command."
            ),
        ),
        DeclareLaunchArgument(
            "capability_profile",
            default_value="obstacle_detection",
            choices=["obstacle_detection", "no_obstacle_detection"],
        ),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run only the Gazebo server when true.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="Start local RViz diagnostics with guarded 2D goal control.",
        ),
        DeclareLaunchArgument(
            "world",
            default_value=str(
                Path(get_package_share_directory("salus_simulation"))
                / "worlds" / "empty.world"
            ),
        ),
        DeclareLaunchArgument("use_keepout", default_value="true"),
        DeclareLaunchArgument("launch_web", default_value="true"),
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("web_ws_port", default_value="8766"),
        DeclareLaunchArgument("web_telemetry_profile", default_value="compact"),
        DeclareLaunchArgument(
            "runtime_dir", default_value="runtime/sim_operational",
            description="Single writable root for this profile's persistent state.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(integration)),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "vehicle_io_profile": LaunchConfiguration("vehicle_io_profile"),
                "compare_legacy_odometry": LaunchConfiguration("compare_legacy_odometry"),
                "command_input_mode": LaunchConfiguration("command_input_mode"),
                "capability_profile": LaunchConfiguration("capability_profile"),
                "gz_args": gz_args,
                "world": LaunchConfiguration("world"),
                "rviz": LaunchConfiguration("rviz"),
                "launch_navigation": "true",
                "use_keepout": LaunchConfiguration("use_keepout"),
                "zones_runtime_dir": PathJoinSubstitution([runtime_dir, "zones"]),
                "launch_routes": "true",
                "launch_patrol": "true",
                "patrol_runtime_dir": PathJoinSubstitution([runtime_dir, "patrol"]),
                "launch_web": LaunchConfiguration("launch_web"),
                "launch_camera": LaunchConfiguration("launch_camera"),
                "web_ws_port": LaunchConfiguration("web_ws_port"),
                "web_waypoints_file": PathJoinSubstitution([
                    runtime_dir, "web", "waypoints.yaml"
                ]),
                "web_telemetry_profile": LaunchConfiguration("web_telemetry_profile"),
                "camera_presets_file": PathJoinSubstitution([
                    runtime_dir, "camera", "presets.json"
                ]),
            }.items(),
        ),
    ])
