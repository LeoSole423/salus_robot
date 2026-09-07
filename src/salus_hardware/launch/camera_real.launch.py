"""Launch the real PTZ owner without owning video transport."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("camera_host", default_value=""),
        DeclareLaunchArgument("camera_port", default_value="0"),
        DeclareLaunchArgument("camera_channel", default_value="0"),
        DeclareLaunchArgument(
            "camera_presets_file",
            default_value="/ros2_ws/log/runtime/camera/presets.json",
        ),
        Node(
            package="salus_hardware",
            executable="camera_node",
            name="salus_camera",
            output="screen",
            parameters=[{
                "backend": "isapi",
                "camera_host": LaunchConfiguration("camera_host"),
                "camera_port": ParameterValue(
                    LaunchConfiguration("camera_port"), value_type=int
                ),
                "camera_channel": ParameterValue(
                    LaunchConfiguration("camera_channel"), value_type=int
                ),
                "camera_presets_file": LaunchConfiguration(
                    "camera_presets_file"
                ),
                "use_sim_time": False,
            }],
        ),
    ])
