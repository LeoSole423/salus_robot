"""Launch the software-only real Nav2 core and its causal startup coordinator."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_keepout = LaunchConfiguration("use_keepout")
    config_dir = Path(get_package_share_directory("salus_navigation")) / "config"
    config = str(config_dir / "nav2_core_real.yaml")
    bt_xml = str(config_dir / "navigation_core.xml")
    through_poses_xml = str(config_dir / "navigation_through_poses_inactive.xml")
    nodes = [
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
    ]
    params = [config, {
        "use_sim_time": False,
        "default_nav_to_pose_bt_xml": bt_xml,
        "default_nav_through_poses_bt_xml": through_poses_xml,
    }]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    actions = [
        DeclareLaunchArgument("use_keepout", default_value="true"),
    ]
    actions.extend(
        Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=params,
            remappings=remappings,
        )
        for package, executable, name in nodes
    )
    actions.extend([
        Node(
            package="salus_navigation",
            executable="nav_observer",
            name="nav_observer",
            output="screen",
            parameters=[{"use_sim_time": False}],
        ),
        Node(
            package="salus_navigation",
            executable="path_health",
            name="path_health",
            output="screen",
            parameters=[{"use_sim_time": False, "costmap_timeout_s": 5.0}],
        ),
        Node(
            package="salus_navigation",
            executable="navigation_profile_coordinator",
            name="navigation_profile_coordinator",
            output="screen",
            parameters=[{"use_sim_time": False}],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "autostart": False,
                "node_names": [name for _, _, name in nodes],
            }],
        ),
        Node(
            package="salus_navigation",
            executable="nav2_startup_coordinator",
            name="nav2_startup_coordinator",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "require_clock_progress": False,
                "use_keepout": ParameterValue(use_keepout, value_type=bool),
                "obstacle_detection_required": True,
            }],
        ),
    ])
    return LaunchDescription(actions)
