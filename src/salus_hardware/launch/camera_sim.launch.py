"""Isolated simulated PTZ camera; intentionally does not start video."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_presets_file",
            default_value="runtime/camera/presets.json",
        ),
        Node(
            package="salus_hardware",
            executable="camera_node",
            name="salus_camera",
            output="screen",
            parameters=[{
                "backend": "sim",
                "camera_presets_file": LaunchConfiguration(
                    "camera_presets_file"
                ),
            }],
        ),
    ])
