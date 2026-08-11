"""Composable local-only localization for the isolated motion simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share_dir = Path(get_package_share_directory("salus_localization"))
    params_file = share_dir / "config" / "localization_local_sim.yaml"
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="salus_localization",
                executable="sim_imu_from_odom",
                name="sim_imu_from_odom",
                output="screen",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            ),
            Node(
                package="salus_localization",
                executable="imu_normalizer",
                name="imu_normalizer",
                output="screen",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
            ),
            Node(
                package="salus_localization",
                executable="ackermann_odometry",
                name="ackermann_odometry",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "invert_measured_steer_sign": True,
                    }
                ],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node_local",
                output="screen",
                parameters=[str(params_file), {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
                remappings=[("odometry/filtered", "/odometry/local")],
            ),
        ]
    )
