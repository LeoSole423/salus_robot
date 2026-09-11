from pathlib import Path
from xml.etree import ElementTree

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from salus_interfaces.srv import SetNavGoalLL
from salus_interfaces.msg import ProjectedKeepoutPolygon, ProjectedKeepoutState
from salus_navigation.nav_command_server import NavCommandServer, projected_keepouts_contain


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


def test_goal_contract_accepts_finite_multi_pose_chunks() -> None:
    request = _request()
    request.lats = [1.0, 2.0]
    request.lons = [3.0, 4.0]
    request.yaws_deg = [5.0, 6.0]

    assert NavCommandServer._waypoints(request) == (
        ((1.0, 3.0, 5.0), (2.0, 4.0, 6.0)),
        "",
    )
    assert "multiple" in NavCommandServer._single_waypoint(request)[1]

    poses = [PoseStamped(), PoseStamped()]
    multi_goal = NavCommandServer._action_goal(poses)
    assert isinstance(multi_goal, NavigateThroughPoses.Goal)
    assert len(multi_goal.poses) == 2


def test_single_pose_still_uses_navigate_to_pose() -> None:
    pose = PoseStamped()
    goal = NavCommandServer._action_goal([pose])
    assert isinstance(goal, NavigateToPose.Goal)
    assert goal.pose is pose


def test_goal_contract_rejects_loops_and_invalid_values() -> None:
    request = _request()
    request.loop = True
    assert "finite" in NavCommandServer._waypoints(request)[1]
    request.loop = False
    request.lats, request.lons, request.yaws_deg = [1.0, 2.0], [3.0, 4.0], [0.0, 0.0]
    assert NavCommandServer._waypoints(request)[1] == ""
    request.lats, request.lons, request.yaws_deg = [1.0], [], [0.0]
    assert "equal" in NavCommandServer._waypoints(request)[1]


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


def test_vector_goal_rejection_handles_holes_and_long_range_coordinates() -> None:
    state = ProjectedKeepoutState()
    state.header.frame_id = "map"
    polygon = ProjectedKeepoutPolygon()
    from geometry_msgs.msg import Point32
    for x, y in ((1049.0, -1.0), (1051.0, -1.0), (1051.0, 1.0), (1049.0, 1.0)):
        polygon.outer.points.append(Point32(x=x, y=y))
    state.polygons.append(polygon)
    assert projected_keepouts_contain(state, 1050.0, 0.0)
    assert not projected_keepouts_contain(state, 0.0, 0.0)


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
    assert "vector_keepout_layer" in config
    for profile in (config, no_obstacles_config):
        assert "smooth_path: false" in profile
        assert "ConstrainedSmoother" not in profile
        assert "smoother_server" not in profile
    source = (ROOT / "salus_navigation" / "nav_command_server.py").read_text(encoding="utf-8")
    assert '"/zones_manager/projected_keepouts"' in source
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
    assert tree.count('server_timeout="500"') == 5
    assert 'hz="0.333"' in tree
    assert "NavigateToPose" not in tree
    assert "SmoothPath" not in tree
    assert "smoothed_path" not in tree
    assert 'path="{candidate_path}"' in tree
    assert 'input_path="{candidate_path}" output_path="{path}"' in tree
    assert '<RecoveryNode number_of_retries="1" name="FollowPathRecovery">' in tree
    assert '<FollowPath path="{path}" controller_id="FollowPath" server_timeout="500"/>' in tree
    assert '<Wait wait_duration="1"/>' in tree
    assert "Spin" not in tree and "BackUp" not in tree


def test_multi_pose_navigator_uses_stable_candidate_validation_and_ackermann_recovery() -> None:
    for profile_name in (
        "nav2_core_sim.yaml",
        "nav2_core_no_obstacles_sim.yaml",
        "nav2_core_real.yaml",
    ):
        profile = (ROOT / "config" / profile_name).read_text(encoding="utf-8")
        assert "navigators: [navigate_to_pose, navigate_through_poses]" in profile
        assert "nav2_bt_navigator::NavigateThroughPosesNavigator" in profile
        assert "nav2_remove_passed_goals_action_bt_node" in profile

    tree = (ROOT / "config" / "navigation_through_poses.xml").read_text(
        encoding="utf-8"
    )
    assert '<RemovePassedGoals input_goals="{goals}" output_goals="{goals}" radius="2.5"/>' in tree
    assert '<ComputePathThroughPoses goals="{goals}" path="{candidate_path}" planner_id="GridBased"/>' in tree
    assert '<CopyPath input_path="{candidate_path}" output_path="{path}"/>' in tree
    assert 'context="0" expected_state="2"' in tree
    assert "global_costmap/clear_entirely_global_costmap" in tree
    assert "local_costmap/clear_entirely_local_costmap" in tree
    assert "Spin" not in tree and "BackUp" not in tree and "SmoothPath" not in tree


def test_navigation_launches_select_the_production_multi_pose_tree() -> None:
    for launch_name in ("navigation_core_sim.launch.py", "navigation_core_real.launch.py"):
        source = (ROOT / "launch" / launch_name).read_text(encoding="utf-8")
        assert "navigation_through_poses.xml" in source
        assert "navigation_through_poses_inactive.xml" not in source


def test_stop_and_wait_cannot_release_a_stale_path_to_controller() -> None:
    root = ElementTree.parse(ROOT / "config" / "navigation_core.xml").getroot()
    stop_and_wait = root.find(
        ".//ReactiveSequence[@name='StopAndWaitForPathData']"
    )

    assert stop_and_wait is not None
    assert [child.tag for child in stop_and_wait] == [
        "IsPathHealthValid",
        "Wait",
        "AlwaysFailure",
    ]
    assert stop_and_wait[-1].tag == "AlwaysFailure"


def test_startup_coordinator_keeps_lifecycle_activation_causal() -> None:
    source = (ROOT / "salus_navigation" / "nav2_startup_coordinator.py").read_text(
        encoding="utf-8"
    )
    assert '"/odometry/global"' in source
    assert '"/scan_clean"' in source
    assert '"/keepout_filter_mask"' not in source
    assert 'lookup_transform("map", "base_footprint"' in source
    assert "ManageLifecycleNodes.Request.STARTUP" in source
    assert 'EvaluatePathHealth, "/path_health/evaluate"' in source
    assert "path_health_preflight" in source
    assert '"planner_server", "controller_server", "bt_navigator", "behavior_server"' in source
    assert "smoother_server" not in source
