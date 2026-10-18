"""Simulation-only sensitivity: inject 300 ms GNSS latency.

Never install or select as a product bringup. All I/O remains simulated.
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    integration=Path(get_package_share_directory('salus_bringup'))/'launch/integration_sim.launch.py'
    return LaunchDescription([
        SetParameter(name='gnss.latency_s',value=0.3),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(integration))),
    ])
