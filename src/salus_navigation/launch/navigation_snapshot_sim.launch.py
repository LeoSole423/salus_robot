"""Partial launch for the navigation snapshot server."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="salus_navigation", executable="nav_snapshot_server",
             name="nav_snapshot_server", output="screen",
             parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]),
    ])
