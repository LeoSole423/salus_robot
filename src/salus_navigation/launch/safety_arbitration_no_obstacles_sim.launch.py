"""Explicitly relay commands without local obstacle detection in simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="salus_navigation",
            executable="safety_command_passthrough",
            name="safety_command_passthrough",
            output="screen",
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
        Node(
            package="salus_navigation",
            executable="nav_command_server",
            name="nav_command_server",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "forward_cmd_vel_safe_without_goal": True,
                "obstacle_detection_required": False,
            }],
        ),
    ])
