"""Launch the physical RS16 owner without perception or other hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = LaunchConfiguration("config_path")
    default_config = PathJoinSubstitution(
        [FindPackageShare("salus_hardware"), "config", "rs16.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=default_config,
                description="Path to the RS16 rslidar_sdk configuration YAML",
            ),
            Node(
                package="rslidar_sdk",
                executable="rslidar_sdk_node",
                name="rslidar_sdk_node",
                output="screen",
                parameters=[{"config_path": config_path}],
            ),
        ]
    )
