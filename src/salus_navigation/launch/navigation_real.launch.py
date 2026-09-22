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
    patrol_runtime_dir = LaunchConfiguration("patrol_runtime_dir")
    patrol_battery_guard_topic = LaunchConfiguration("patrol_battery_guard_topic")
    patrol_battery_state_topic = LaunchConfiguration("patrol_battery_state_topic")
    route_execution_mode = LaunchConfiguration("route_execution_mode")
    adaptive_dense_leg_max_m = LaunchConfiguration("adaptive_dense_leg_max_m")
    adaptive_dense_horizon_m = LaunchConfiguration("adaptive_dense_horizon_m")
    route_progress_pose_max_age_s = LaunchConfiguration(
        "route_progress_pose_max_age_s")
    zones_launch = PathJoinSubstitution([
        package_share, "launch", "navigation_zones_real.launch.py",
    ])
    collision_launch = PathJoinSubstitution([
        package_share, "launch", "collision_monitor_real.launch.py",
    ])
    core_launch = PathJoinSubstitution([
        package_share, "launch", "navigation_core_real.launch.py",
    ])
    route_launch = PathJoinSubstitution([
        package_share, "launch", "route_executor_real.launch.py",
    ])
    patrol_launch = PathJoinSubstitution([
        package_share, "launch", "patrol_mission_real.launch.py",
    ])
    snapshot_launch = PathJoinSubstitution([
        package_share, "launch", "navigation_snapshot_real.launch.py",
    ])
    return LaunchDescription([
        DeclareLaunchArgument("use_keepout", default_value="true"),
        DeclareLaunchArgument("zones_runtime_dir", default_value="runtime/zones"),
        DeclareLaunchArgument("patrol_runtime_dir", default_value="runtime/patrol"),
        DeclareLaunchArgument(
            "patrol_battery_guard_topic", default_value="/battery_mission_guard"),
        DeclareLaunchArgument(
            "patrol_battery_state_topic", default_value="/battery_state"),
        DeclareLaunchArgument(
            "route_execution_mode",
            default_value="legacy_pair",
            choices=["single_checkpoint", "legacy_pair", "adaptive_dense"],
        ),
        DeclareLaunchArgument("adaptive_dense_leg_max_m", default_value="8.0"),
        DeclareLaunchArgument("adaptive_dense_horizon_m", default_value="35.0"),
        DeclareLaunchArgument(
            "route_progress_pose_max_age_s", default_value="0.5"),
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(route_launch),
            launch_arguments={
                "use_sim_time": "false",
                "route_execution_mode": route_execution_mode,
                "adaptive_dense_leg_max_m": adaptive_dense_leg_max_m,
                "adaptive_dense_horizon_m": adaptive_dense_horizon_m,
                "route_progress_pose_max_age_s": route_progress_pose_max_age_s,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(patrol_launch),
            launch_arguments={
                "use_sim_time": "false",
                "runtime_dir": patrol_runtime_dir,
                "battery_guard_topic": patrol_battery_guard_topic,
                "battery_state_topic": patrol_battery_state_topic,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(snapshot_launch),
            launch_arguments={"use_sim_time": "false"}.items(),
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
