"""
Single-authority UART controller profile for the physical MVP cutover.

This launch deliberately contains only the controller that consumes the
authoritative legacy ``/cmd_vel_final`` contract and owns the ESP32 UART. It
must never be composed with read-only or shadow profiles.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="auto"),
            Node(
                package="salus_control",
                executable="controller_server_node",
                name="salus_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "transport_backend": "uart",
                        "command_input_mode": "legacy_cmd_vel",
                        "serial_port": serial_port,
                        "serial_baud": 115200,
                        "serial_tx_hz": 50.0,
                        "max_reverse_mps": 1.30,
                        "wheelbase_m": 0.94,
                        "steering_limit_rad": 0.5235987756,
                        "operational_steering_limit_rad": 0.3141592654,
                        "manual_operational_steering_limit_rad": 0.5235987756,
                        "vx_deadband_mps": 0.10,
                        "vx_min_effective_mps": 0.75,
                        "invert_steer_from_cmd_vel": True,
                    }
                ],
            ),
        ]
    )
