"""Launch the GeoJSON owner that publishes map-frame vector keepouts."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_keepout = LaunchConfiguration("use_keepout")
    runtime_dir = LaunchConfiguration("runtime_dir")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_keepout", default_value="true"),
        DeclareLaunchArgument("runtime_dir", default_value="runtime/zones"),
        Node(
            package="salus_navigation", executable="zones_manager", name="zones_manager", output="screen",
            parameters=[{
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "runtime_dir": runtime_dir,
                "use_keepout": ParameterValue(use_keepout, value_type=bool),
                "service_timeout_s": 4.0,
            }],
        ),
    ])
