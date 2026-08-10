"""Launch the isolated Ackermann motion simulation for SALUS."""

from pathlib import Path
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn_robot(context, *, xacro_file: Path):
    """Materialize Xacro for Gazebo, which accepts a model file rather than a parameter."""
    with tempfile.NamedTemporaryFile(
        prefix="salus_ackermann_", suffix=".urdf", delete=False
    ) as temporary_file:
        robot_file = temporary_file.name
    subprocess.run(
        ["xacro", str(xacro_file), "use_sim:=true", "-o", robot_file],
        check=True,
    )
    return [
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_salus_ackermann",
            output="screen",
            arguments=[
                "-name", LaunchConfiguration("model_name").perform(context),
                "-file", robot_file,
                "-x", LaunchConfiguration("spawn_x").perform(context),
                "-y", LaunchConfiguration("spawn_y").perform(context),
                "-z", LaunchConfiguration("spawn_z").perform(context),
                "-Y", LaunchConfiguration("spawn_yaw").perform(context),
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    simulation_share = Path(get_package_share_directory("salus_simulation"))
    description_share = Path(get_package_share_directory("salus_description"))
    ros_gz_sim_share = Path(get_package_share_directory("ros_gz_sim"))
    xacro_file = description_share / "urdf" / "salus_robot.urdf.xacro"
    world_file = simulation_share / "worlds" / "empty.world"
    robot_description = ParameterValue(
        Command(["xacro ", str(xacro_file), " use_sim:=true"]),
        value_type=str,
    )

    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world", default_value=str(world_file)),
            DeclareLaunchArgument("model_name", default_value="salus_ackermann"),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.30"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("gz_args", default_value="-r -s"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(ros_gz_sim_share / "launch" / "gz_sim.launch.py")
                ),
                launch_arguments={
                    "gz_args": [LaunchConfiguration("gz_args"), " ", world]
                }.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": robot_description,
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="motion_bridge",
                output="screen",
                parameters=[
                    {
                        "config_file": str(
                            simulation_share / "config" / "motion_bridge.yaml"
                        )
                    }
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="joint_state_bridge",
                output="screen",
                arguments=[
                    "/world/salus_empty/model/salus_ackermann/joint_state"
                    "@sensor_msgs/msg/JointState[gz.msgs.Model"
                ],
                remappings=[
                    ("/world/salus_empty/model/salus_ackermann/joint_state", "/joint_states")
                ],
            ),
            OpaqueFunction(
                function=_spawn_robot, kwargs={"xacro_file": xacro_file}
            ),
        ]
    )
