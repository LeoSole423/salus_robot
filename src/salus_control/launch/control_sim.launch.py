"""Launch the migrated controller with its non-hardware simulation backend."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="salus_control",
                executable="controller_server_node",
                name="controller_server",
                output="screen",
                parameters=[
                    {
                        "transport_backend": "sim_gazebo",
                        "serial_port": "/dev/null",
                        "sim_cmd_vel_topic": "/cmd_vel_gazebo",
                        "sim_odom_topic": "/odom_raw",
                        "sim_joint_states_topic": "/joint_states",
                    }
                ],
            )
        ]
    )
