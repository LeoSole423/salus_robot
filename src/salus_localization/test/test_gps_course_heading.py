import json
import math
import time

import rclpy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from salus_localization.gps_course_heading import GpsCourseHeading


LAT = -31.4858037
LON = -64.2410570


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def fix(stamp_s: int, east_m: float) -> NavSatFix:
    message = NavSatFix()
    message.header.stamp.sec = stamp_s
    message.latitude = LAT
    message.longitude = LON + east_m / (
        111_320.0 * math.cos(math.radians(LAT))
    )
    return message


def test_course_heading_evaluates_on_each_new_gps_fix() -> None:
    rclpy.init()
    node = GpsCourseHeading()
    try:
        output = CapturingPublisher()
        debug = CapturingPublisher()
        node.output = output
        node.debug = debug
        node.speed = 1.2
        node.yaw_rate = 0.0
        node.steer = 0.0
        node.steer_valid = True
        node.rtk_status = "rtk_fixed"
        node.rtk_at_monotonic = time.monotonic()

        node.on_fix(fix(1, 0.0))
        assert debug.messages
        first = json.loads(debug.messages[-1].data)
        assert first["valid"] is False
        assert first["reason"] == "distance_below_threshold"

        node.on_fix(fix(2, 3.0))
        second = json.loads(debug.messages[-1].data)
        assert second["valid"] is True
        assert second["reason"] == "ok"
        assert len(output.messages) == 1
        assert output.messages[0].header.stamp.sec == 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
