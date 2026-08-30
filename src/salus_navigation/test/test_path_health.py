"""Characterization tests for stable-path clearance and replanning policy."""

from array import array
from types import SimpleNamespace

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from salus_interfaces.msg import PathHealth

from salus_navigation.path_health import CostmapView, PathHealthPolicy, costmap_view_from_message


def make_path(points):
    message = Path(); message.header.frame_id = "map"
    for x, y in points:
        pose = PoseStamped(); pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        message.poses.append(pose)
    return message


def costmap(costs=(), stamp=10.0, resolution=0.25):
    width = height = 100
    data = [0] * (width * height)
    for x, y, cost in costs:
        data[y * width + x] = cost
    return CostmapView(resolution, width, height, 0.0, 0.0, stamp, tuple(data))


def test_empty_costmap_keeps_path_and_small_error_does_not_replan():
    policy = PathHealthPolicy(cross_track_confirmations=2)
    result = policy.evaluate(make_path([(1, 1), (12, 1)]), robot_x=1, robot_y=1.3, costmap=costmap(), now_s=10.0)
    assert result.state == PathHealth.KEEP_PATH
    assert result.reason == "path_healthy"


def test_cell_cost_preserves_cost_values_for_non_tuple_backing_sequence():
    policy = PathHealthPolicy()
    view = CostmapView(1.0, 3, 2, 10.0, 20.0, 10.0, array("B", [0, 42, 252, 253, 254, 255]))

    assert [policy._cell_cost(view, x, y) for x, y in (
        (10.0, 20.0), (11.0, 20.0), (12.0, 20.0),
        (10.0, 21.0), (11.0, 21.0), (12.0, 21.0),
    )] == [0, 42, 252, 253, 254, 255]


def test_cell_cost_returns_free_for_coordinates_outside_costmap():
    policy = PathHealthPolicy()
    view = CostmapView(0.5, 2, 2, 10.0, 20.0, 10.0, [253, 254, 255, 100])

    assert policy._cell_cost(view, 9.99, 20.0) == 0
    assert policy._cell_cost(view, 10.0, 19.99) == 0
    assert policy._cell_cost(view, 11.0, 20.0) == 0
    assert policy._cell_cost(view, 10.0, 21.0) == 0


def test_costmap_adapter_keeps_ros_sequence_identity():
    data = array("B", [0, 253, 254, 255])
    message = SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=500_000_000)),
        metadata=SimpleNamespace(
            resolution=0.5,
            size_x=2,
            size_y=2,
            origin=SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0)),
        ),
        data=data,
    )

    view = costmap_view_from_message(message)

    assert view.data is data
    assert view.stamp_s == 10.5
    assert PathHealthPolicy._cell_cost(view, 1.5, 2.5) == 255


def test_lethal_cost_forces_replan():
    policy = PathHealthPolicy()
    result = policy.evaluate(make_path([(1, 1), (12, 1)]), robot_x=1, robot_y=1, costmap=costmap([(16, 4, 254)]), now_s=10.0)
    assert result.state == PathHealth.REPLAN
    assert result.reason == "path_collision"


def test_sustained_inflation_forces_replan_but_single_sample_does_not():
    path = make_path([(1, 1), (12, 1)])
    single = PathHealthPolicy().evaluate(path, robot_x=1, robot_y=1, costmap=costmap([(16, 4, 120)]), now_s=10.0)
    sustained = PathHealthPolicy().evaluate(path, robot_x=1, robot_y=1, costmap=costmap([(16, 4, 120), (17, 4, 120), (18, 4, 120)]), now_s=10.0)
    assert single.state == PathHealth.KEEP_PATH
    assert sustained.state == PathHealth.REPLAN
    assert sustained.reason == "clearance_degraded"


def test_cross_track_requires_persistence_then_respects_cooldown():
    policy = PathHealthPolicy(cross_track_confirmations=2, cooldown_s=1.5)
    path = make_path([(1, 1), (12, 1)])
    first = policy.evaluate(path, robot_x=2, robot_y=2.0, costmap=costmap(), now_s=10.0)
    second = policy.evaluate(path, robot_x=2, robot_y=2.0, costmap=costmap(), now_s=10.2)
    third = policy.evaluate(path, robot_x=2, robot_y=2.0, costmap=costmap(), now_s=10.3)
    assert first.state == PathHealth.KEEP_PATH
    assert second.reason == "cross_track_error"
    assert third.state == PathHealth.KEEP_PATH


def test_stale_or_missing_data_stops_and_waits():
    policy = PathHealthPolicy(costmap_timeout_s=1.5)
    path = make_path([(1, 1), (12, 1)])
    stale = policy.evaluate(path, robot_x=1, robot_y=1, costmap=costmap(stamp=1.0), now_s=10.0)
    missing = policy.evaluate(path, robot_x=1, robot_y=1, costmap=None, now_s=10.0)
    no_tf = policy.evaluate(path, robot_x=1, robot_y=1, costmap=costmap(), now_s=10.0, tf_available=False)
    assert stale.state == missing.state == no_tf.state == PathHealth.STOP_AND_WAIT


def test_progress_stall_forces_replan_after_timeout():
    policy = PathHealthPolicy(progress_timeout_s=2.0)
    path = make_path([(1, 1), (12, 1)])
    policy.evaluate(path, robot_x=1, robot_y=1, costmap=costmap(), now_s=10.0)
    result = policy.evaluate(path, robot_x=1, robot_y=1, costmap=costmap(stamp=12.5), now_s=12.5)
    assert result.state == PathHealth.REPLAN
    assert result.reason == "progress_stalled"


def test_candidate_does_not_mutate_active_path_progress_or_hysteresis():
    policy = PathHealthPolicy(cross_track_confirmations=2, progress_timeout_s=2.0)
    active = make_path([(1, 1), (12, 1)])
    policy.evaluate(active, robot_x=1, robot_y=1, costmap=costmap(), now_s=10.0)
    candidate = make_path([(1, 1), (12, 2)])
    result = policy.evaluate(
        candidate, robot_x=1, robot_y=1, costmap=costmap(stamp=12.5), now_s=12.5,
        track_active_state=False)
    assert result.state == PathHealth.KEEP_PATH
    active_result = policy.evaluate(active, robot_x=1, robot_y=1, costmap=costmap(stamp=12.6), now_s=12.6)
    assert active_result.reason == "progress_stalled"
