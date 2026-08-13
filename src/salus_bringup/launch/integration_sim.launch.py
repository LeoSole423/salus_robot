"""Integrated simulation checkpoint for the subsystems migrated so far."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include(package: str, launch_file: str, arguments=None, condition=None):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")
    rviz = LaunchConfiguration("rviz")
    launch_navigation = LaunchConfiguration("launch_navigation")
    use_keepout = LaunchConfiguration("use_keepout")
    zones_runtime_dir = LaunchConfiguration("zones_runtime_dir")

    common = {"use_sim_time": use_sim_time}
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r -s",
                description="Gazebo arguments; use '-r' for the graphical client.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="false",
                description="Start the on-demand LiDAR RViz diagnostics window.",
            ),
            DeclareLaunchArgument(
                "launch_navigation",
                default_value="true",
                description="Start the Nav2 autonomous navigation core.",
            ),
            DeclareLaunchArgument(
                "use_keepout",
                default_value="true",
                description="Enable dynamic GeoJSON keepout costmap filters.",
            ),
            DeclareLaunchArgument(
                "zones_runtime_dir",
                default_value="runtime/zones",
                description="Runtime directory for the dynamic keepout mask.",
            ),
            _include(
                "salus_simulation",
                "motion_sim.launch.py",
                {"use_sim_time": use_sim_time, "gz_args": gz_args},
            ),
            _include("salus_control", "control_sim.launch.py", common),
            _include("salus_localization", "localization_sim.launch.py", common),
            _include(
                "salus_localization",
                "global_localization_sim.launch.py",
                common,
            ),
            _include("salus_perception", "lidar_sim.launch.py"),
            _include("salus_navigation", "safety_arbitration_sim.launch.py", common),
            _include(
                "salus_navigation",
                "navigation_zones_sim.launch.py",
                {
                    "use_sim_time": use_sim_time,
                    "use_keepout": use_keepout,
                    "runtime_dir": zones_runtime_dir,
                },
                condition=IfCondition(launch_navigation),
            ),
            _include(
                "salus_navigation",
                "navigation_core_sim.launch.py",
                common,
                condition=IfCondition(launch_navigation),
            ),
            _include(
                "salus_perception",
                "lidar_diagnostics.launch.py",
                condition=IfCondition(rviz),
            ),
        ]
    )
