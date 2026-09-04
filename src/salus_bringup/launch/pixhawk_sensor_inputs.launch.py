"""Read-only Pixhawk/MAVROS sensor composition; never starts actuation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    imu_input = LaunchConfiguration("imu_input_topic")
    gnss_input = LaunchConfiguration("gnss_input_topic")
    sensor_frame = LaunchConfiguration("sensor_frame")
    imu_expected_frame = LaunchConfiguration("imu_expected_frame")
    gnss_expected_frame = LaunchConfiguration("gnss_expected_frame")
    return LaunchDescription([
        DeclareLaunchArgument("imu_input_topic", default_value="/imu/data"),
        DeclareLaunchArgument(
            "gnss_input_topic", default_value="/global_position/raw/fix"
        ),
        DeclareLaunchArgument("sensor_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "imu_expected_frame", default_value=sensor_frame
        ),
        DeclareLaunchArgument(
            "gnss_expected_frame", default_value=sensor_frame
        ),
        Node(
            package="salus_hardware",
            executable="pixhawk_sensor_adapter",
            name="pixhawk_sensor_adapter",
            output="screen",
            parameters=[{
                "imu_input_topic": imu_input,
                "imu_output_topic": "/hardware/imu_primary/data",
                "imu_expected_frame": imu_expected_frame,
                "gnss_input_topic": gnss_input,
                "gnss_output_topic": "/hardware/gnss_primary/fix",
                "gnss_expected_frame": gnss_expected_frame,
            }],
        ),
        Node(
            package="salus_localization",
            executable="imu_selector",
            name="imu_selector",
            output="screen",
            parameters=[{
                "selected_source": "imu_primary",
                "primary_topic": "/hardware/imu_primary/data",
                "primary_frame": imu_expected_frame,
                "output_topic": "/salus/imu/data",
            }],
        ),
        Node(
            package="salus_localization",
            executable="gnss_selector",
            name="gnss_selector",
            output="screen",
            parameters=[{
                "selected_source": "gnss_primary",
                "primary_topic": "/hardware/gnss_primary/fix",
                "primary_frame": gnss_expected_frame,
                "output_topic": "/salus/gps/fix",
            }],
        ),
    ])
