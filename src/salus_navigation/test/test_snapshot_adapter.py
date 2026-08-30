import math

import pytest
from geometry_msgs.msg import Point, Point32
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from salus_navigation.nav_snapshot_server import NavSnapshotServer, apply_grid_update, is_fresh
from salus_navigation.snapshot_renderer import Transform2D


def test_freshness_separates_startup_grace_age_and_future_data() -> None:
    assert is_fresh(0, 0, 10.0, 14.9, 2.0, 5.0)
    assert not is_fresh(0, 0, 10.0, 15.1, 2.0, 5.0)
    assert is_fresh(9_000_000_000, 10_000_000_000, 0.0, 100.0, 2.0, 5.0)
    assert not is_fresh(7_000_000_000, 10_000_000_000, 0.0, 100.0, 2.0, 5.0)
    assert not is_fresh(11_000_000_000, 10_000_000_000, 0.0, 100.0, 2.0, 5.0)


def test_incremental_costmap_update_is_applied_without_mutating_base() -> None:
    grid = OccupancyGrid()
    grid.info.width = 4
    grid.info.height = 3
    grid.data = [0] * 12
    update = OccupancyGridUpdate()
    update.x, update.y = 1, 1
    update.width, update.height = 2, 2
    update.data = [10, 20, 30, 40]
    update.header.stamp.sec = 7

    result = apply_grid_update(grid, update)

    assert result is not None
    assert list(result.data) == [0, 0, 0, 0, 0, 10, 20, 0, 0, 30, 40, 0]
    assert list(grid.data) == [0] * 12
    assert result.header.stamp.sec == 7


@pytest.mark.parametrize("x,y,width,height,data", [
    (-1, 0, 1, 1, [1]),
    (3, 0, 2, 1, [1, 2]),
    (0, 2, 1, 2, [1, 2]),
    (0, 0, 2, 2, [1]),
])
def test_invalid_incremental_costmap_update_is_rejected(x, y, width, height, data) -> None:
    grid = OccupancyGrid()
    grid.info.width = 4
    grid.info.height = 3
    grid.data = [0] * 12
    update = OccupancyGridUpdate(x=x, y=y, width=width, height=height, data=data)
    assert apply_grid_update(grid, update) is None


def test_collision_markers_apply_pose_rotation_and_preserve_line_list_pairs() -> None:
    server = object.__new__(NavSnapshotServer)
    server._parameter = {"base_frame": "base_footprint"}
    server._transform = lambda target, source, stamp: Transform2D(source, target, 0.0, 0.0, 0.0)
    marker = Marker()
    marker.header.frame_id = "base_footprint"
    marker.type = Marker.LINE_LIST
    marker.pose.position.x = 2.0
    marker.pose.position.y = 3.0
    marker.pose.orientation.z = math.sin(math.pi / 4.0)
    marker.pose.orientation.w = math.cos(math.pi / 4.0)
    marker.points = [Point(x=1.0, y=0.0), Point(x=2.0, y=0.0),
                     Point(x=0.0, y=1.0), Point(x=0.0, y=2.0)]
    lines = server._collision(MarkerArray(markers=[marker]), "odom")
    assert len(lines) == 2
    assert lines[0].points[0] == pytest.approx((2.0, 4.0))
    assert lines[0].points[1] == pytest.approx((2.0, 5.0))
    assert lines[1].points[0] == pytest.approx((1.0, 3.0))
    assert lines[1].points[1] == pytest.approx((0.0, 3.0))


def test_old_projected_keepouts_use_current_tf_for_localization_corrections() -> None:
    from salus_interfaces.msg import ProjectedKeepoutPolygon, ProjectedKeepoutState

    state = ProjectedKeepoutState()
    state.header.frame_id = "map"
    state.header.stamp.sec = 1  # Deliberately older than the TF cache horizon.
    polygon = ProjectedKeepoutPolygon()
    polygon.outer.points = [Point32(x=10.0, y=0.0), Point32(x=11.0, y=0.0), Point32(x=10.0, y=1.0)]
    state.polygons = [polygon]
    calls = []
    server = object.__new__(NavSnapshotServer)
    server._transform = lambda target, source, stamp: calls.append(stamp) or Transform2D(source, target, -50.0, 0.0, 0.0)

    keepouts = server._keepouts(state, "odom")

    assert len(keepouts) == 1
    assert keepouts[0].transform.x == -50.0
    assert calls[0].nanoseconds == 0
