"""Optional route executor over the already-running navigation stack."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    route_execution_mode = LaunchConfiguration("route_execution_mode")
    route_progress_pose_max_age_s = LaunchConfiguration(
        "route_progress_pose_max_age_s")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "route_execution_mode",
            default_value="single_checkpoint",
            choices=["single_checkpoint", "legacy_pair"],
        ),
        DeclareLaunchArgument(
            "route_progress_pose_max_age_s", default_value="0.5"),
        Node(
            package="salus_navigation",
            executable="route_executor",
            name="route_executor",
            output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "route_execution_mode": route_execution_mode,
                "route_progress_pose_max_age_s": ParameterValue(
                    route_progress_pose_max_age_s, value_type=float),
            }],
        ),
    ])
