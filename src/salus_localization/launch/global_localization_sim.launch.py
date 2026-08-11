"""Global GPS localization; requires the local partial launch and motion simulation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = str(Path(get_package_share_directory("salus_localization")) / "config" / "localization_global_sim.yaml")
    params = [{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]
    return LaunchDescription([DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="salus_localization", executable="sim_gps_normalizer", name="sim_gps_normalizer", output="screen", parameters=params),
        Node(package="salus_localization", executable="gps_course_heading", name="gps_course_heading", output="screen", parameters=params),
        Node(package="salus_localization", executable="map_gps_absolute_measurement", name="map_gps_absolute_measurement", output="screen", parameters=params),
        Node(package="robot_localization", executable="navsat_transform_node", name="navsat_transform", output="screen", parameters=[config, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}], remappings=[("gps/fix", "/gps/fix"), ("odometry/filtered", "/odometry/local"), ("odometry/gps", "/odometry/gps")]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node_global", output="screen", parameters=[config, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}], remappings=[("odometry/filtered", "/odometry/global")])])
