"""Canonical vehicle measurement and conversion chain for simulation only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time = ParameterValue(use_sim_time, value_type=bool)
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="salus_hardware",
                executable="legacy_drive_measurement_node",
                name="legacy_drive_measurement_adapter",
                output="screen",
                parameters=[{"use_sim_time": sim_time}],
            ),
            Node(
                package="salus_hardware",
                executable="vehicle_kinematic_converter",
                name="vehicle_kinematic_converter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": sim_time,
                        # Validated against the simulator contract only. These are
                        # not calibration values for the physical robot.
                        "calibration_validated": True,
                        "traction_linear_scale": 1.0,
                        "steering_coefficients": [0.0, -1.0],
                    }
                ],
            ),
        ]
    )
