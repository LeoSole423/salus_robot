"""Real RS16 perception pipeline, without owning the sensor or TF."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="salus_perception",
                executable="scan_ground_filter",
                name="scan_ground_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/scan_3d",
                        "output_topic": "/obstacles_cloud",
                        "target_frame": "base_footprint",
                        "wheelbase_m": 0.94,
                        "profile": "urban",
                        "ground_tolerance_m": 0.20,
                        "range_max": 20.0,
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                remappings=[
                    ("cloud_in", "/obstacles_cloud"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "target_frame": "base_footprint",
                        "transform_tolerance": 0.1,
                        "min_height": -0.1,
                        "max_height": 1.6,
                        "angle_min": -1.5707963,
                        "angle_max": 1.5707963,
                        "angle_increment": 0.00872665,
                        "range_min": 0.4,
                        "range_max": 20.0,
                        "use_inf": True,
                        "use_sim_time": False,
                    }
                ],
            ),
            Node(
                package="salus_perception",
                executable="scan_noise_filter",
                name="scan_noise_filter",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/scan",
                        "output_topic": "/scan_clean",
                        "range_min": 0.4,
                        "range_max": 20.0,
                        "speckle_window": 2,
                        "speckle_max_range": 12.0,
                        "max_deviation_m": 0.30,
                        "use_sim_time": False,
                    }
                ],
            ),
        ]
    )
