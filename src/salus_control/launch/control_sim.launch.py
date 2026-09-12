"""Launch the migrated controller with its non-hardware simulation backend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    command_input_mode = LaunchConfiguration("command_input_mode")
    sim_sensor_profile = LaunchConfiguration("sim_sensor_profile")
    sim_sensor_seed = LaunchConfiguration("sim_sensor_seed")
    sensor_profile_file = PathJoinSubstitution([
        FindPackageShare("salus_simulation"),
        "config",
        "sensor_profiles",
        PythonExpression(["'", sim_sensor_profile, "'.lower() + '.yaml'"]),
    ])
    perturbed_condition = IfCondition(
        PythonExpression(["'", sim_sensor_profile, "'.lower() != 'clean'"])
    )
    sim_odom_topic = PythonExpression([
        "'/sim/sensors/drive_odom' if '",
        sim_sensor_profile,
        "'.lower() != 'clean' else '/odom_raw'",
    ])
    sim_joint_states_topic = PythonExpression([
        "'/sim/sensors/joint_states' if '",
        sim_sensor_profile,
        "'.lower() != 'clean' else '/joint_states'",
    ])
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "command_input_mode", default_value="legacy_cmd_vel"
            ),
            DeclareLaunchArgument(
                "sim_sensor_profile",
                default_value="clean",
                choices=["clean", "independent_nominal", "degraded"],
                description="Simulation-only drive sensor perturbation profile.",
            ),
            DeclareLaunchArgument(
                "sim_sensor_seed",
                default_value="6400",
                description="Deterministic base seed for simulated sensor streams.",
            ),
            Node(
                package="salus_control",
                executable="sim_drive_sensor_adapter",
                name="sim_drive_sensor_adapter",
                output="screen",
                parameters=[
                    sensor_profile_file,
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "sim_sensor_profile": sim_sensor_profile,
                        "sim_sensor_seed": ParameterValue(sim_sensor_seed, value_type=int),
                    }
                ],
                condition=perturbed_condition,
            ),
            Node(
                package="salus_control",
                executable="controller_server_node",
                name="salus_controller",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                        "transport_backend": "sim_gazebo",
                        "command_input_mode": command_input_mode,
                        "serial_port": "/dev/null",
                        "sim_cmd_vel_topic": "/cmd_vel_gazebo",
                        "sim_odom_topic": sim_odom_topic,
                        "sim_joint_states_topic": sim_joint_states_topic,
                        "sim_invert_actuation_steer_sign": False,
                    }
                ],
            ),
            Node(
                package="salus_control",
                executable="legacy_vehicle_command_node",
                name="legacy_vehicle_command_adapter",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
            ),
            Node(
                package="salus_control",
                executable="vehicle_command_comparison_node",
                name="vehicle_command_shadow_comparison",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
            ),
            Node(
                package="salus_control",
                executable="canonical_command_dry_run_node",
                name="canonical_command_dry_run",
                output="screen",
                parameters=[
                    {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}
                ],
            ),
        ]
    )
