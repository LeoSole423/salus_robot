"""Start the single MAVROS Pixhawk owner for the physical MVP."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    hardware_share = get_package_share_directory("salus_hardware")
    mavros_share = get_package_share_directory("mavros")
    fcu_url = LaunchConfiguration("fcu_url")
    pluginlists = os.path.join(
        hardware_share, "config", "mavros_sensor_only_pluginlists.yaml"
    )
    overrides = os.path.join(hardware_share, "config", "mavros_apm_overrides.yaml")
    apm_config = os.path.join(mavros_share, "launch", "apm_config.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:921600"),
            Node(
                package="mavros",
                executable="mavros_node",
                name="mavros_node",
                output="screen",
                remappings=[
                    ("mavros_node/data", "/imu/data"),
                    ("mavros_node/raw/fix", "/global_position/raw/fix"),
                    (
                        "mavros_node/velocity_local",
                        "/local_position/velocity_local",
                    ),
                    ("mavros_node/odom", "/local_position/odom"),
                ],
                parameters=[
                    pluginlists,
                    apm_config,
                    overrides,
                    {
                        "fcu_url": fcu_url,
                        "gcs_url": "",
                        "tgt_system": 1,
                        "tgt_component": 1,
                        "fcu_protocol": "v2.0",
                    },
                ],
            ),
        ]
    )
