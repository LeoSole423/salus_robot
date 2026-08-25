"""Launch only the non-authoritative navigation evaluation observer."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="observe"),
        DeclareLaunchArgument("scenario", default_value=""),
        DeclareLaunchArgument("output_dir"),
        DeclareLaunchArgument(
            "goal_tolerance_m", default_value="1.2",
            description="Current functional Nav2 arrival tolerance in metres.",
        ),
        DeclareLaunchArgument(
            "precision_target_m", default_value="0.25",
            description="Report-only future arrival precision target in metres.",
        ),
        DeclareLaunchArgument("observe_timeout_s", default_value="90.0"),
        Node(package="salus_evaluation", executable="navigation_evaluation",
             name="navigation_evaluation", output="screen", parameters=[{
                 "use_sim_time": True, "mode": LaunchConfiguration("mode"),
                 "scenario": LaunchConfiguration("scenario"),
                 "output_dir": LaunchConfiguration("output_dir"),
                 "goal_tolerance_m": LaunchConfiguration("goal_tolerance_m"),
                 "precision_target_m": LaunchConfiguration("precision_target_m"),
                 "observe_timeout_s": LaunchConfiguration("observe_timeout_s"),
             }]),
    ])
