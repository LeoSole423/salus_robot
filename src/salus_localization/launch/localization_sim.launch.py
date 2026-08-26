"""Composable local-only localization for the isolated motion simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    share_dir = Path(get_package_share_directory("salus_localization"))
    params_file = share_dir / "config" / "localization_local_sim.yaml"
    use_sim_time = LaunchConfiguration("use_sim_time")
    odometry_backend = LaunchConfiguration("odometry_backend")
    compare_legacy_odometry = LaunchConfiguration("compare_legacy_odometry")
    legacy_condition = IfCondition(
        PythonExpression(["'", odometry_backend, "' == 'legacy'"])
    )
    canonical_condition = IfCondition(
        PythonExpression(["'", odometry_backend, "' == 'canonical'"])
    )
    shadow_condition = IfCondition(
        PythonExpression(
            [
                "'", odometry_backend, "' == 'canonical' and '",
                compare_legacy_odometry, "'.lower() == 'true'",
            ]
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "odometry_backend",
                default_value="legacy",
                description="Wheel odometry authority: legacy or canonical.",
            ),
            DeclareLaunchArgument(
                "compare_legacy_odometry",
                default_value="false",
                description="Publish legacy odometry on isolated comparison topics.",
            ),
            OpaqueFunction(function=_validate_profile),
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
                condition=legacy_condition,
            ),
            Node(
                package="salus_localization",
                executable="kinematic_ackermann_odometry",
                name="kinematic_ackermann_odometry",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
                condition=canonical_condition,
            ),
            Node(
                package="salus_localization",
                executable="ackermann_odometry",
                name="legacy_ackermann_odometry_shadow",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "invert_measured_steer_sign": True,
                        "odom_topic": "/comparison/legacy/wheel_odometry",
                        "twist_topic": "/comparison/legacy/vehicle_twist",
                    }
                ],
                condition=shadow_condition,
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


def _validate_profile(context, *args, **kwargs):
    backend = LaunchConfiguration("odometry_backend").perform(context).strip()
    compare = LaunchConfiguration("compare_legacy_odometry").perform(context).lower()
    if backend not in {"legacy", "canonical"}:
        raise ValueError("odometry_backend must be 'legacy' or 'canonical'")
    if compare not in {"true", "false"}:
        raise ValueError("compare_legacy_odometry must be 'true' or 'false'")
    if backend == "legacy" and compare == "true":
        raise ValueError("legacy comparison is only valid with canonical odometry")
    return []
