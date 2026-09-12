"""Canonical vehicle measurement and conversion chain for simulation only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_sensor_profile = LaunchConfiguration("sim_sensor_profile")
    sim_sensor_seed = LaunchConfiguration("sim_sensor_seed")
    sim_time = ParameterValue(use_sim_time, value_type=bool)
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
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
                package="salus_hardware",
                executable="legacy_drive_measurement_node",
                name="legacy_drive_measurement_adapter",
                output="screen",
                parameters=[{
                    "use_sim_time": sim_time,
                    "sim_sensor_profile": sim_sensor_profile,
                    "sim_sensor_seed": ParameterValue(sim_sensor_seed, value_type=int),
                }],
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
                        "sim_sensor_profile": sim_sensor_profile,
                        "sim_sensor_seed": ParameterValue(sim_sensor_seed, value_type=int),
                    }
                ],
            ),
        ]
    )
