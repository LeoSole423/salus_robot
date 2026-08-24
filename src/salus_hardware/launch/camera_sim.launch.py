"""Isolated simulated PTZ camera; intentionally does not start video."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="salus_hardware",
            executable="camera_node",
            name="salus_camera",
            output="screen",
            parameters=[{"backend": "sim"}],
        ),
    ])
