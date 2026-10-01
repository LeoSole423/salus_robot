"""Isolated read-only battery backend, with no drive or actuation nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('battery_backend', default_value='pylontech_us2000'),
        DeclareLaunchArgument('battery_serial_port'),
        DeclareLaunchArgument('battery_baud', default_value='115200'),
        DeclareLaunchArgument('battery_address', default_value='2'),
        Node(package='salus_hardware', executable='battery_node', output='screen',
             parameters=[{
                 'backend': LaunchConfiguration('battery_backend'),
                 'serial_port': LaunchConfiguration('battery_serial_port'),
                 'baud': ParameterValue(LaunchConfiguration('battery_baud'), value_type=int),
                 'address': ParameterValue(LaunchConfiguration('battery_address'), value_type=int),
             }]),
    ])
