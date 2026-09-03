"""Start the single NTRIP acquisition owner for the physical MVP."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_path"),
            DeclareLaunchArgument("active_source_id", default_value=""),
            Node(
                package="salus_hardware",
                executable="ntrip_rtcm_source",
                name="ntrip_rtcm_source",
                output="screen",
                parameters=[
                    {
                        "sources_config": LaunchConfiguration("config_path"),
                        "active_source_id": LaunchConfiguration("active_source_id"),
                    }
                ],
            ),
        ]
    )
