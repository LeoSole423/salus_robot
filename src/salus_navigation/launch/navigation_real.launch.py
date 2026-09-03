"""Compose the software-only real navigation, safety and arbitration profile."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("salus_navigation")
    use_keepout = LaunchConfiguration("use_keepout")
    zones_runtime_dir = LaunchConfiguration("zones_runtime_dir")
    zones_launch = PathJoinSubstitution([
        package_share, "launch", "navigation_zones_real.launch.py",
    ])
    collision_launch = PathJoinSubstitution([
        package_share, "launch", "collision_monitor_real.launch.py",
    ])
    core_launch = PathJoinSubstitution([
        package_share, "launch", "navigation_core_real.launch.py",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("use_keepout", default_value="true"),
        DeclareLaunchArgument("zones_runtime_dir", default_value="runtime/zones"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(zones_launch),
            launch_arguments={
                "use_sim_time": "false",
                "use_keepout": use_keepout,
                "runtime_dir": zones_runtime_dir,
            }.items(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(collision_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={"use_keepout": use_keepout}.items(),
        ),
        Node(
            package="salus_navigation",
            executable="nav_command_server",
            name="nav_command_server",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "cmd_vel_safe_topic": "/cmd_vel_safe",
                "cmd_vel_final_topic": "/cmd_vel_final",
                "safety_scan_topic": "/scan_clean",
                "gps_topic": "/salus/gps/fix",
                "obstacle_detection_required": True,
                "fromll_service": "/fromLL",
            }],
        ),
    ])
