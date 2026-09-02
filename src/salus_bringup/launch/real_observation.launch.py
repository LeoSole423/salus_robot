"""
Read-only coexistence profile for the deployed legacy robot.

This composition only observes topics already published by ``ROS2_SALUS``.
It deliberately has no launch arguments for RTCM delivery: corrections always
end in the dry-run sink while the legacy bridge owns the Pixhawk path.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package: str, launch_file: str, arguments: dict[str, object]):
    share = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / launch_file)),
        launch_arguments=arguments.items(),
    )


def generate_launch_description() -> LaunchDescription:
    """Compose only adapters which cannot open hardware or command it."""
    imu_input_topic = LaunchConfiguration("imu_input_topic")
    gnss_input_topic = LaunchConfiguration("gnss_input_topic")
    sensor_frame = LaunchConfiguration("sensor_frame")
    legacy_status_topic = LaunchConfiguration("legacy_status_topic")
    legacy_fix_topic = LaunchConfiguration("legacy_fix_topic")
    legacy_rtcm_topic = LaunchConfiguration("legacy_rtcm_topic")
    gpsraw_topic = LaunchConfiguration("gpsraw_topic")
    legacy_telemetry_topic = LaunchConfiguration("legacy_telemetry_topic")
    legacy_command_topic = LaunchConfiguration("legacy_command_topic")

    return LaunchDescription([
        DeclareLaunchArgument("imu_input_topic", default_value="/imu/data"),
        DeclareLaunchArgument(
            "gnss_input_topic", default_value="/global_position/raw/fix"
        ),
        DeclareLaunchArgument("sensor_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "legacy_status_topic", default_value="/gps/rtk_source/status_json"
        ),
        DeclareLaunchArgument(
            "legacy_fix_topic", default_value="/gps/rtk_status_mavros"
        ),
        DeclareLaunchArgument("legacy_rtcm_topic", default_value="/rtcm"),
        DeclareLaunchArgument(
            "gpsraw_topic", default_value="/mavros_node/gps1/raw"
        ),
        DeclareLaunchArgument(
            "legacy_telemetry_topic",
            default_value="/controller/drive_telemetry",
        ),
        DeclareLaunchArgument("legacy_command_topic", default_value="/cmd_vel_final"),
        _include(
            "salus_bringup",
            "pixhawk_sensor_inputs.launch.py",
            {
                "imu_input_topic": imu_input_topic,
                "gnss_input_topic": gnss_input_topic,
                "sensor_frame": sensor_frame,
            },
        ),
        _include(
            "salus_bringup",
            "rtk_gnss_observation.launch.py",
            {
                "enabled": "true",
                "legacy_status_topic": legacy_status_topic,
                "legacy_fix_topic": legacy_fix_topic,
                "legacy_rtcm_topic": legacy_rtcm_topic,
                "gpsraw_topic": gpsraw_topic,
                "delivery_backend": "disabled",
                "delivery_enabled": "false",
                "legacy_rtcm_type": "uint8_multi_array",
            },
        ),
        Node(
            package="salus_hardware",
            executable="legacy_drive_measurement_node",
            name="legacy_drive_measurement_adapter",
            namespace="/salus/observation",
            output="screen",
            parameters=[{
                "legacy_telemetry_topic": legacy_telemetry_topic,
                "input_wire_type": "interfaces",
                "traction_topic": "/vehicle/measurements/traction",
                "steering_topic": "/vehicle/measurements/steering",
                "traction_source_id": "rear_traction_motor",
                "steering_source_id": "front_steering_linkage",
            }],
        ),
        Node(
            package="salus_control",
            executable="legacy_vehicle_command_node",
            name="legacy_vehicle_command_adapter",
            namespace="/salus/observation",
            output="screen",
            parameters=[{
                "input_topic": legacy_command_topic,
                "input_wire_type": "interfaces",
                "output_topic": "/vehicle/command_shadow",
                "frame_id": "base_footprint",
            }],
        ),
        Node(
            package="salus_control",
            executable="vehicle_command_comparison_node",
            name="vehicle_command_shadow_comparison",
            namespace="/salus/observation",
            output="screen",
            parameters=[{
                "legacy_topic": legacy_command_topic,
                "input_wire_type": "interfaces",
                "shadow_topic": "/vehicle/command_shadow",
                "diagnostics_topic": "/vehicle/command_shadow/diagnostics",
                "frame_id": "base_footprint",
            }],
        ),
    ])
