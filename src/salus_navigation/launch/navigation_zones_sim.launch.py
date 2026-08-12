"""Launch dynamic keepout filters and their GeoJSON owner for simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_keepout = LaunchConfiguration("use_keepout")
    runtime_dir = LaunchConfiguration("runtime_dir")
    config_dir = Path(get_package_share_directory("salus_navigation")) / "config"
    filter_nodes = [
        Node(
            package="nav2_map_server", executable="map_server", name="keepout_filter_mask_server", output="screen",
            parameters=[{"yaml_filename": str(config_dir / "keepout_empty.yaml"), "topic_name": "/keepout_filter_mask", "frame_id": "map", "use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
        Node(
            package="nav2_map_server", executable="costmap_filter_info_server", name="keepout_costmap_filter_info_server", output="screen",
            parameters=[{"type": 0, "filter_info_topic": "/costmap_filter_info", "mask_topic": "/keepout_filter_mask", "base": 0.0, "multiplier": 1.0, "use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_keepout", default_value="true"),
        DeclareLaunchArgument("runtime_dir", default_value="runtime/zones"),
        *filter_nodes,
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_keepout_filters", output="screen",
            parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool), "autostart": True, "node_names": ["keepout_filter_mask_server", "keepout_costmap_filter_info_server"]}],
        ),
        TimerAction(
            period=3.0,
            actions=[Node(
                package="salus_navigation", executable="zones_manager", name="zones_manager", output="screen",
                parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool), "runtime_dir": runtime_dir, "use_keepout": ParameterValue(use_keepout, value_type=bool)}],
            )],
        ),
    ])
