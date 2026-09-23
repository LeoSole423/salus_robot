"""Global GPS localization; requires the local partial launch and motion simulation."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from salus_localization.datum_profile import validate_datum_override


def _build_datum_nodes(context):
    """Apply one explicit simulation datum to `/fromLL` and navsat_transform."""
    use_sim_time = LaunchConfiguration("use_sim_time")
    datum_lat, datum_lon, datum_yaw_deg = validate_datum_override(
        float(LaunchConfiguration("datum_lat").perform(context)),
        float(LaunchConfiguration("datum_lon").perform(context)),
        float(LaunchConfiguration("datum_yaw_deg").perform(context)),
    )
    config = str(
        Path(get_package_share_directory("salus_localization"))
        / "config"
        / "localization_global_sim.yaml"
    )
    sim_time = ParameterValue(use_sim_time, value_type=bool)
    return [
        Node(
            package="salus_localization",
            executable="map_gps_absolute_measurement",
            name="map_gps_absolute_measurement",
            output="screen",
            parameters=[
                {"use_sim_time": sim_time},
                {
                    "datum_lat": datum_lat,
                    "datum_lon": datum_lon,
                    "datum_yaw_deg": datum_yaw_deg,
                },
            ],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                config,
                {
                    "use_sim_time": sim_time,
                    "datum": [datum_lat, datum_lon, math.radians(datum_yaw_deg)],
                },
            ],
            remappings=[
                ("imu/data", "/localization/orientation"),
                ("gps/fix", "/gps/fix"),
                ("odometry/filtered", "/odometry/local"),
                ("odometry/gps", "/odometry/gps"),
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    orientation_source = LaunchConfiguration("orientation_source")
    global_ekf_params_file = LaunchConfiguration("global_ekf_params_file")
    sim_sensor_profile = LaunchConfiguration("sim_sensor_profile")
    sim_sensor_seed = LaunchConfiguration("sim_sensor_seed")
    sensor_profile_file = PathJoinSubstitution([
        FindPackageShare("salus_simulation"),
        "config",
        "sensor_profiles",
        PythonExpression(["'", sim_sensor_profile, "'.lower() + '.yaml'"]),
    ])
    config = str(
        Path(get_package_share_directory("salus_localization"))
        / "config"
        / "localization_global_sim.yaml"
    )
    params = [{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}]
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("datum_lat", default_value="-31.4858037"),
        DeclareLaunchArgument("datum_lon", default_value="-64.2410570"),
        DeclareLaunchArgument("datum_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument(
            "global_ekf_params_file",
            default_value=config,
            description=(
                "Simulation-only global EKF YAML; defaults to the current baseline."
            ),
        ),
        DeclareLaunchArgument(
            "orientation_source",
            default_value="course_over_ground",
            choices=["course_over_ground", "external_heading"],
            description=(
                "Exclusive global orientation authority. Missing data never selects "
                "the other source."
            ),
        ),
        DeclareLaunchArgument(
            "sim_sensor_profile",
            default_value="clean",
            choices=["clean", "independent_nominal", "degraded"],
            description="Simulation-only sensor profile.",
        ),
        DeclareLaunchArgument(
            "sim_sensor_seed",
            default_value="6400",
            description="Non-negative deterministic simulation sensor seed.",
        ),
        Node(
            package="salus_localization",
            executable="sim_gps_normalizer",
            name="sim_gps_normalizer",
            output="screen",
            parameters=[
                sensor_profile_file,
                *params,
                {
                    "sim_sensor_profile": sim_sensor_profile,
                    "sim_sensor_seed": ParameterValue(sim_sensor_seed, value_type=int),
                },
            ],
        ),
        Node(
            package="salus_localization",
            executable="global_stationary_gates",
            name="global_stationary_gates",
            output="screen",
            parameters=params,
        ),
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
            parameters=[*params, {"selected_source": orientation_source}],
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
        OpaqueFunction(function=_build_datum_nodes),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node_global",
            output="screen",
            parameters=[
                global_ekf_params_file,
                {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
            ],
            remappings=[("odometry/filtered", "/odometry/global")],
        ),
    ])
