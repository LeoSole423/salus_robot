"""Read-only BMI088/Pixhawk observation; starts no control, MAVROS or EKF."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declarations = [
        DeclareLaunchArgument('port', description='Explicit S3 device, never auto'),
        DeclareLaunchArgument('device_id', default_value=''),
        DeclareLaunchArgument('calibration_path', default_value=''),
        DeclareLaunchArgument('reference_topic', default_value='/hardware/imu_primary/data'),
        DeclareLaunchArgument('mount_configured', default_value='false',
                              description='Publish mounting TF only after measuring/declaring it'),
    ]
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        declarations.append(DeclareLaunchArgument('mount_' + key, default_value='0.0'))
    tf_arguments = []
    for key in ('x', 'y', 'z', 'roll', 'pitch', 'yaw'):
        tf_arguments += ['--' + key, LaunchConfiguration('mount_' + key)]
    tf_arguments += ['--frame-id', 'base_footprint', '--child-frame-id', 'bmi088_link']
    return LaunchDescription(declarations + [
        Node(package='salus_hardware', executable='bmi088_observer', output='screen',
             parameters=[{key: ParameterValue(LaunchConfiguration(key), value_type=str)
                          for key in ('port', 'device_id', 'calibration_path')}]),
        Node(package='salus_hardware', executable='imu_comparison', output='screen',
             parameters=[{'reference_topic': LaunchConfiguration('reference_topic')}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='bmi088_mount', arguments=tf_arguments,
             condition=IfCondition(LaunchConfiguration('mount_configured'))),
    ])
