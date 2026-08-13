"""Launch the minimal autonomous Nav2 stack for the simulated Ackermann robot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_dir = Path(get_package_share_directory("salus_navigation")) / "config"
    config = config_dir / "nav2_core_sim.yaml"
    params = [config, {
        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
        "default_nav_to_pose_bt_xml": str(config_dir / "navigation_core.xml"),
        "default_nav_through_poses_bt_xml": str(config_dir / "navigation_through_poses_inactive.xml"),
    }]
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nodes = [
        ("nav2_planner", "planner_server", "planner_server"),
        ("nav2_controller", "controller_server", "controller_server"),
        ("nav2_smoother", "smoother_server", "smoother_server"),
        ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ("nav2_behaviors", "behavior_server", "behavior_server"),
    ]
    actions = [DeclareLaunchArgument("use_sim_time", default_value="true")]
    actions.extend(
        Node(package=package, executable=executable, name=name, output="screen", parameters=params, remappings=remappings)
        for package, executable, name in nodes
    )
    actions.append(Node(
        package="salus_navigation", executable="nav_observer", name="nav_observer",
        output="screen", parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    ))
    actions.append(Node(
        package="salus_navigation", executable="path_health", name="path_health",
        output="screen", parameters=[{
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            # Costmap publication can briefly stall while Gazebo/keepout
            # reloads.  Five seconds still fails safe, but avoids treating a
            # healthy startup or atomic mask reload as a permanent obstacle.
            "costmap_timeout_s": 5.0,
        }],
    ))
    actions.append(Node(
        package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
        output="screen", parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool), "autostart": True,
                                        "node_names": [name for _, _, name in nodes]}],
    ))
    return LaunchDescription(actions)
