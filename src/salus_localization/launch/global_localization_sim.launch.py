"""Global GPS localization; requires the local partial launch and motion simulation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    orientation_source = LaunchConfiguration("orientation_source")
    config = str(Path(get_package_share_directory("salus_localization")) / "config" / "localization_global_sim.yaml")
    params = [{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]
    return LaunchDescription([DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "orientation_source",
            default_value="course_over_ground",
            choices=["course_over_ground", "external_heading"],
            description=(
                "Exclusive global orientation authority. Missing data never selects "
                "the other source."
            ),
        ),
        Node(package="salus_localization", executable="sim_gps_normalizer", name="sim_gps_normalizer", output="screen", parameters=params),
        Node(package="salus_localization", executable="global_stationary_gates", name="global_stationary_gates", output="screen", parameters=params),
        Node(
            package="salus_localization",
            executable="gps_course_heading",
            name="gps_course_heading",
            output="screen",
            parameters=params,
            condition=IfCondition(PythonExpression([
                "'", orientation_source, "' == 'course_over_ground'",
            ])),
        ),
        Node(
            package="salus_localization",
            executable="orientation_source_selector",
            name="orientation_source_selector",
            output="screen",
            parameters=[
                *params,
                {"selected_source": orientation_source},
            ],
        ),
        Node(
            package="salus_localization",
            executable="sim_external_heading_from_odom",
            name="sim_external_heading_from_odom",
            output="screen",
            parameters=params,
            condition=IfCondition(PythonExpression([
                "'", orientation_source, "' == 'external_heading'",
            ])),
        ),
        Node(package="salus_localization", executable="map_gps_absolute_measurement", name="map_gps_absolute_measurement", output="screen", parameters=params),
        Node(package="robot_localization", executable="navsat_transform_node", name="navsat_transform", output="screen", parameters=[config, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}], remappings=[("imu/data", "/localization/orientation"), ("gps/fix", "/gps/fix"), ("odometry/filtered", "/odometry/local"), ("odometry/gps", "/odometry/gps")]),
        Node(package="robot_localization", executable="ekf_node", name="ekf_filter_node_global", output="screen", parameters=[config, {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}], remappings=[("odometry/filtered", "/odometry/global")])])
