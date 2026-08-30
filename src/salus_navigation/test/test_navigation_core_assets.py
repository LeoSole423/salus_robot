from pathlib import Path

from geometry_msgs.msg import PoseStamped
from salus_interfaces.srv import SetNavGoalLL
from salus_navigation.nav_command_server import NavCommandServer


ROOT = Path(__file__).parents[1]


def _request() -> SetNavGoalLL.Request:
    request = SetNavGoalLL.Request()
    request.lat = -31.4858037
    request.lon = -64.2410570
    request.yaw_deg = 0.0
    return request


def test_single_goal_contract_accepts_scalar_or_one_element_arrays() -> None:
    scalar, error = NavCommandServer._single_waypoint(_request())
    assert scalar == (-31.4858037, -64.2410570, 0.0)
    assert error == ""
    request = _request()
    request.lats, request.lons, request.yaws_deg = [1.0], [2.0], [3.0]
    assert NavCommandServer._single_waypoint(request) == ((1.0, 2.0, 3.0), "")


def test_single_goal_contract_rejects_missions_and_invalid_values() -> None:
    request = _request()
    request.loop = True
    assert "missions" in NavCommandServer._single_waypoint(request)[1]
    request.loop = False
    request.lats, request.lons, request.yaws_deg = [1.0, 2.0], [3.0, 4.0], [0.0, 0.0]
    assert "multiple" in NavCommandServer._single_waypoint(request)[1]
    request.lats, request.lons, request.yaws_deg = [1.0], [], [0.0]
    assert "equal" in NavCommandServer._single_waypoint(request)[1]


def test_rviz_goal_contract_accepts_a_finite_map_pose() -> None:
    message = PoseStamped()
    message.header.frame_id = "map"
    message.pose.position.x = 4.0
    message.pose.position.y = -2.0
    message.pose.orientation.z = 2**-0.5
    message.pose.orientation.w = 2**-0.5
    goal, error = NavCommandServer._rviz_map_goal(message)
    assert error == ""
    assert goal[:2] == (4.0, -2.0)
    assert abs(goal[2] - 90.0) < 1e-9


def test_rviz_goal_contract_rejects_wrong_frame_and_invalid_orientation() -> None:
    message = PoseStamped()
    message.header.frame_id = "odom"
    message.pose.orientation.w = 1.0
    assert "map frame" in NavCommandServer._rviz_map_goal(message)[1]
    message.header.frame_id = "map"
    message.pose.orientation.w = 0.0
    assert "orientation" in NavCommandServer._rviz_map_goal(message)[1]


def test_navigation_config_and_launch_keep_the_safe_contract() -> None:
    config = (ROOT / "config" / "nav2_core_sim.yaml").read_text(encoding="utf-8")
    no_obstacles_config = (
        ROOT / "config" / "nav2_core_no_obstacles_sim.yaml"
    ).read_text(encoding="utf-8")
    launch = (ROOT / "launch" / "navigation_core_sim.launch.py").read_text(encoding="utf-8")
    tree = (ROOT / "config" / "navigation_core.xml").read_text(encoding="utf-8")
    coordinator = (
        ROOT / "salus_navigation" / "nav2_startup_coordinator.py"
    ).read_text(encoding="utf-8")
    observer = (ROOT / "salus_navigation" / "nav_observer.py").read_text(
        encoding="utf-8"
    )
    package = (ROOT / "package.xml").read_text(encoding="utf-8")
    assert "SmacPlannerHybrid" in config
    assert "RegulatedPurePursuitController" in config
    assert "/scan_clean" in config
    assert "keepout_filter" in config
    for profile in (config, no_obstacles_config):
        assert "smooth_path: false" in profile
        assert "ConstrainedSmoother" not in profile
        assert "smoother_server" not in profile
    source = (ROOT / "salus_navigation" / "nav_command_server.py").read_text(encoding="utf-8")
    assert '"/keepout_filter_mask"' in source
    assert "TRANSIENT_LOCAL" in source
    assert "lifecycle_manager" in launch
    assert '"autostart": False' in launch
    assert "nav2_startup_coordinator" in launch
    assert "navigation_profile_coordinator" in launch
    assert "nav_observer" in launch
    assert "path_health" in launch
    assert "smoother_server" not in launch
    assert "smoother_server" not in coordinator
    assert "smoother_server" not in observer
    assert "nav2_smoother" not in package
    assert "/path_health/evaluate" in tree
    assert 'context="1"' in tree
    assert "IsPathHealthValid" in tree
    assert tree.count('server_timeout="500"') == 4
    assert 'hz="0.333"' in tree
    assert "NavigateToPose" not in tree
    assert "SmoothPath" not in tree
    assert "smoothed_path" not in tree
    assert 'path="{candidate_path}"' in tree
    assert 'input_path="{candidate_path}" output_path="{path}"' in tree
    assert '<FollowPath path="{path}" controller_id="FollowPath" server_timeout="500"/>' in tree
    assert "Spin" not in tree and "BackUp" not in tree


def test_startup_coordinator_keeps_lifecycle_activation_causal() -> None:
    source = (ROOT / "salus_navigation" / "nav2_startup_coordinator.py").read_text(
        encoding="utf-8"
    )
    assert '"/odometry/global"' in source
    assert '"/scan_clean"' in source
    assert '"/keepout_filter_mask"' in source
    assert 'lookup_transform("map", "base_footprint"' in source
    assert "ManageLifecycleNodes.Request.STARTUP" in source
    assert 'EvaluatePathHealth, "/path_health/evaluate"' in source
    assert "path_health_preflight" in source
    assert '"planner_server", "controller_server", "bt_navigator", "behavior_server"' in source
    assert "smoother_server" not in source
