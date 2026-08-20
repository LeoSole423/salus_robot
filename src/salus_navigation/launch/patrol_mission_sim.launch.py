"""Optional structured patrol/HOME coordinator over the route executor."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    runtime_dir = LaunchConfiguration("runtime_dir")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("runtime_dir", default_value="runtime/patrol"),
        Node(package="salus_navigation", executable="patrol_mission_coordinator",
             name="patrol_mission_coordinator", output="screen", parameters=[{
                 "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                 "runtime_dir": runtime_dir,
             }]),
    ])
