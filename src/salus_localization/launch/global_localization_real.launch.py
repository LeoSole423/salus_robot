"""Software-only real global localization profile for SALUS."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from salus_localization.datum_profile import (
    resolve_selected_datum,
    validate_datum_override,
)


CONFIG_FILE = "localization_global_real.yaml"


def _build_nodes(context):
    package_share = get_package_share_directory("salus_localization")
    config_file = str(Path(package_share) / "config" / CONFIG_FILE)
    datum_lat, datum_lon, datum_yaw = validate_datum_override(
        float(LaunchConfiguration("datum_lat").perform(context)),
        float(LaunchConfiguration("datum_lon").perform(context)),
        float(LaunchConfiguration("datum_yaw_deg").perform(context)),
    )

    return [
        Node(
            package="salus_localization",
            executable="global_stationary_gates",
            name="global_stationary_gates",
            output="screen",
            parameters=[
                {
                    "odom_topic": "/odometry/local",
                    "imu_topic": "/salus/imu/data",
                    "drive_telemetry_topic": "/controller/drive_telemetry",
                    "stationary_speed_threshold_mps": 0.03,
                    "drive_telemetry_timeout_s": 0.5,
                }
            ],
        ),
        Node(
            package="salus_localization",
            executable="gps_course_heading",
            name="gps_course_heading",
            output="screen",
            parameters=[
                {
                    "gps_topic": "/salus/gps/fix",
                    "odom_topic": "/odometry/local",
                    "drive_telemetry_topic": "/controller/drive_telemetry",
                    "rtk_status_topic": "/salus/hardware/gnss_primary/rtk_status",
                    "rtk_status_wire_type": "gnss_rtk_status",
                    "output_topic": "/gps/course_heading",
                    "base_frame": "base_footprint",
                    "require_rtk": True,
                    "rtk_status_max_age_s": 2.5,
                    "min_distance_m": 2.0,
                    "min_speed_mps": 0.8,
                    "max_abs_steer_deg": 3.0,
                    "max_abs_yaw_rate_rps": 0.05,
                    "max_fix_age_s": 0.5,
                    "max_sample_dt_s": 2.5,
                    "invalid_hold_s": 0.8,
                }
            ],
        ),
        Node(
            package="salus_localization",
            executable="orientation_source_selector",
            name="orientation_source_selector",
            output="screen",
            parameters=[
                {
                    "selected_source": "course_over_ground",
                    "course_topic": "/gps/course_heading",
                    "output_topic": "/localization/orientation",
                    "expected_frame": "base_footprint",
                }
            ],
        ),
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                config_file,
                {
                    "use_sim_time": False,
                    "datum": [datum_lat, datum_lon, datum_yaw],
                },
            ],
            remappings=[
                ("gps/fix", "/salus/gps/fix"),
                ("odometry/filtered", "/odometry/global"),
                ("odometry/gps", "/odometry/gps"),
                ("imu", "/localization/orientation"),
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="salus_global_ekf",
            output="screen",
            parameters=[config_file, {"use_sim_time": False}],
            remappings=[("odometry/filtered", "/odometry/global")],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("salus_localization")
    selected_lat, selected_lon, selected_yaw, _ = resolve_selected_datum(package_share)
    return LaunchDescription(
        [
            DeclareLaunchArgument("datum_lat", default_value=str(selected_lat)),
            DeclareLaunchArgument("datum_lon", default_value=str(selected_lon)),
            DeclareLaunchArgument("datum_yaw_deg", default_value=str(selected_yaw)),
            OpaqueFunction(function=_build_nodes),
        ]
    )
