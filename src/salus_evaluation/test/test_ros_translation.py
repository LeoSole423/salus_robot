from nav_msgs.msg import Odometry
from salus_interfaces.msg import NavEvent

from salus_evaluation.evaluation_runner import _stamp


def test_stamp_accepts_header_and_direct_ros_time_layouts():
    odometry = Odometry()
    odometry.header.stamp.sec = 2
    odometry.header.stamp.nanosec = 500_000_000
    event = NavEvent()
    event.stamp.sec = 3
    event.stamp.nanosec = 250_000_000
    assert _stamp(odometry) == 2.5
    assert _stamp(event) == 3.25
