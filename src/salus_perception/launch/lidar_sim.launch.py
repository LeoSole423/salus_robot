"""Composable 3D LiDAR processing: raw cloud to safety scan."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    share=Path(get_package_share_directory("salus_perception")); collision=str(share / "config" / "collision_monitor.yaml")
    return LaunchDescription([
        Node(package="salus_perception", executable="cloud_normalizer", name="cloud_normalizer", output="screen"),
        Node(package="salus_perception", executable="scan_ground_filter", name="scan_ground_filter", output="screen", parameters=[{"wheelbase_m":0.94}]),
        Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node", name="pointcloud_to_laserscan", output="screen", remappings=[("cloud_in","/obstacles_cloud"),("scan","/scan")], parameters=[{"target_frame":"base_footprint","transform_tolerance":0.1,"min_height":-0.1,"max_height":1.6,"angle_min":-1.5707963,"angle_max":1.5707963,"angle_increment":0.00872665,"range_min":0.4,"range_max":20.0,"use_inf":True}]),
        Node(package="salus_perception", executable="scan_noise_filter", name="scan_noise_filter", output="screen"),
        Node(package="nav2_collision_monitor", executable="collision_monitor", name="collision_monitor", output="screen", parameters=[collision, {"use_sim_time":True}]),
    ])
