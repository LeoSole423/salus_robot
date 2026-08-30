from nav_msgs.msg import OccupancyGrid

from salus_navigation.nav_snapshot_server import NavSnapshotServer


def test_grid_adapter_reuses_ros_occupancy_storage() -> None:
    message = OccupancyGrid()
    message.header.frame_id = "map"
    message.info.resolution = 0.1
    message.info.width = 3
    message.info.height = 2
    message.data = [0, 100, 0, -1, 0, 0]

    grid = NavSnapshotServer._grid(message)

    assert grid is not None
    assert grid.data is message.data
    assert grid.width == 3
    assert grid.height == 2
