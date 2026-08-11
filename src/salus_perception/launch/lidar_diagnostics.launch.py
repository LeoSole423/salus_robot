"""On-demand visual diagnostics. Never required by headless operation."""
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([Node(package="rviz2", executable="rviz2", name="lidar_diagnostics_rviz", output="screen")])
