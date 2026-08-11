"""Activate local collision safety and arbitrate commands for simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = Path(get_package_share_directory("salus_navigation")) / "config" / "collision_monitor.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[config, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_collision_monitor",
            output="screen",
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool), "autostart": True, "node_names": ["collision_monitor"]}],
        ),
        Node(
            package="salus_navigation",
            executable="nav_command_server",
            name="nav_command_server",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "forward_cmd_vel_safe_without_goal": True,
            }],
        ),
    ])
