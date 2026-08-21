"""Optional structured patrol/HOME coordinator over the route executor."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    runtime_dir = LaunchConfiguration("runtime_dir")
    battery_guard_topic = LaunchConfiguration("battery_guard_topic")
    battery_state_topic = LaunchConfiguration("battery_state_topic")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("runtime_dir", default_value="runtime/patrol"),
        DeclareLaunchArgument(
            "battery_guard_topic", default_value="/battery_mission_guard"),
        DeclareLaunchArgument(
            "battery_state_topic", default_value="/battery_state"),
        Node(package="salus_navigation", executable="patrol_mission_coordinator",
             name="patrol_mission_coordinator", output="screen", parameters=[{
                 "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                 "runtime_dir": runtime_dir,
                 "battery_guard_topic": battery_guard_topic,
                 "battery_state_topic": battery_state_topic,
             }]),
    ])
