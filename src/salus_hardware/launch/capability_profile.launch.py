"""Publish the effective, explicitly selected system capability profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile",
            default_value="obstacle_detection",
            choices=["obstacle_detection", "no_obstacle_detection"],
        ),
        Node(
            package="salus_hardware",
            executable="capability_profile",
            name="capability_profile",
            output="screen",
            parameters=[{"profile": LaunchConfiguration("profile")}],
        ),
    ])
