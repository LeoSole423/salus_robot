"""On-demand visual diagnostics. Never required by headless operation."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description() -> LaunchDescription:
    config = Path(get_package_share_directory("salus_perception")) / "config" / "lidar_diagnostics.rviz"
    return LaunchDescription([Node(package="rviz2", executable="rviz2", name="lidar_diagnostics_rviz", output="screen", arguments=["-d", str(config)])])
