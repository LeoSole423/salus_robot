import math

import pytest
from geometry_msgs.msg import TransformStamped
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from salus_perception.scan_ground_filter import ScanGroundFilter


class _IdentityBuffer:
    def lookup_transform(self, target, source, time, *, timeout):
        transform = TransformStamped()
        transform.header.frame_id = target
        transform.child_frame_id = source
        transform.transform.rotation.w = 1.0
        return transform


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _adapter_for_test():
    node = ScanGroundFilter.__new__(ScanGroundFilter)
    node.target = "base_footprint"
    node.tolerance = 0.20
    node.range_max = 20.0
    node.buffer = _IdentityBuffer()
    node.pub = _CapturePublisher()
    return node


@pytest.mark.parametrize(
    ("is_dense", "expected_points"),
    ((False, 1), (True, 2)),
)
def test_adapter_delegates_pointcloud_nan_policy_to_read_points(
    is_dense, expected_points
) -> None:
    header = Header()
    header.stamp.sec = 123
    header.stamp.nanosec = 456
    header.frame_id = "lidar_link"
    message = point_cloud2.create_cloud_xyz32(
        header,
        [(float("nan"), 0.0, 0.0), (2.0, 0.0, 0.5)],
    )
    message.is_dense = is_dense

    node = _adapter_for_test()
    node.on_cloud(message)

    assert len(node.pub.messages) == 1
    output = node.pub.messages[0]
    assert output.header.frame_id == "base_footprint"
    assert (output.header.stamp.sec, output.header.stamp.nanosec) == (123, 456)
    points = list(
        point_cloud2.read_points(
            output, field_names=("x", "y", "z"), skip_nans=False
        )
    )
    assert len(points) == expected_points
    assert any(math.isclose(float(point[0]), 2.0) for point in points)
    assert (is_dense is False) or any(math.isnan(float(point[0])) for point in points)
