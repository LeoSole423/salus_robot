"""Launch the Cockpit-compatible ROS/WebSocket bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("ws_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("ws_port", default_value="8766"),
        DeclareLaunchArgument("enable_control_lock", default_value="true"),
        DeclareLaunchArgument("control_lock_start_locked", default_value="true"),
        DeclareLaunchArgument("control_lock_heartbeat_timeout_s", default_value="2.5"),
        DeclareLaunchArgument("service_timeout_s", default_value="5.0"),
        DeclareLaunchArgument("service_discovery_timeout_s", default_value="5.0"),
        DeclareLaunchArgument("long_service_timeout_s", default_value="20.0"),
        DeclareLaunchArgument("telemetry_profile", default_value="compact"),
        DeclareLaunchArgument("compact_telemetry_hz", default_value="2.0"),
        DeclareLaunchArgument("scan_preview_topic", default_value="/scan_preview"),
        DeclareLaunchArgument("scan_preview_enabled", default_value="true"),
        DeclareLaunchArgument("waypoints_file", default_value="runtime/web/waypoints.yaml"),
        Node(
            package="salus_web",
            executable="web_bridge",
            name="salus_web_gateway",
            output="screen",
            parameters=[{
                "ws_host": LaunchConfiguration("ws_host"),
                "ws_port": LaunchConfiguration("ws_port"),
                "enable_control_lock": LaunchConfiguration("enable_control_lock"),
                "control_lock_start_locked": LaunchConfiguration(
                    "control_lock_start_locked"
                ),
                "control_lock_heartbeat_timeout_s": LaunchConfiguration(
                    "control_lock_heartbeat_timeout_s"
                ),
                "service_timeout_s": LaunchConfiguration("service_timeout_s"),
                "service_discovery_timeout_s": LaunchConfiguration(
                    "service_discovery_timeout_s"
                ),
                "long_service_timeout_s": LaunchConfiguration(
                    "long_service_timeout_s"
                ),
                "telemetry_profile": LaunchConfiguration("telemetry_profile"),
                "compact_telemetry_hz": LaunchConfiguration("compact_telemetry_hz"),
                "scan_preview_topic": LaunchConfiguration("scan_preview_topic"),
                "scan_preview_enabled": LaunchConfiguration("scan_preview_enabled"),
                "waypoints_file": LaunchConfiguration("waypoints_file"),
            }],
        ),
    ])
