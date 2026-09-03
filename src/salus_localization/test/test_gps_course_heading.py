import json
import math
import time

import pytest
import rclpy
from rclpy.parameter import Parameter
from salus_interfaces.msg import GnssRtkStatus
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

from salus_localization.gps_course_heading import GpsCourseHeading


LAT = -31.4858037
LON = -64.2410570
TEST_DRIVE_TOPIC = "/test/gps_course_heading/drive"
TEST_RTK_TOPIC = "/test/gps_course_heading/rtk_status"


class CapturingPublisher:
    def __init__(self, subscription_count=1):
        self.messages = []
        self.subscription_count = subscription_count

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscription_count


def fix(stamp_s: int, east_m: float) -> NavSatFix:
    message = NavSatFix()
    message.header.stamp.sec = stamp_s
    message.latitude = LAT
    message.longitude = LON + east_m / (
        111_320.0 * math.cos(math.radians(LAT))
    )
    return message


def typed_status(fix_quality: int) -> GnssRtkStatus:
    message = GnssRtkStatus()
    message.fix_quality = fix_quality
    message.acquisition_state = message.ACQUISITION_RECEIVING
    message.delivery_state = message.DELIVERY_DELIVERING
    message.corrections_fresh = True
    message.received_count = 42
    return message


def prepare_for_heading(node: GpsCourseHeading) -> None:
    node.output = CapturingPublisher()
    node.debug = CapturingPublisher()
    node.speed = 1.2
    node.yaw_rate = 0.0
    node.steer = 0.0
    node.steer_valid = True


def typed_node() -> GpsCourseHeading:
    return GpsCourseHeading(parameter_overrides=[
        Parameter("rtk_status_wire_type", value="gnss_rtk_status"),
        Parameter("gps_topic", value="/test/gps_course_heading/fix"),
        Parameter("odom_topic", value="/test/gps_course_heading/odom"),
        Parameter("drive_telemetry_topic", value="/test/gps_course_heading/drive"),
        Parameter("rtk_status_topic", value=TEST_RTK_TOPIC),
        Parameter("output_topic", value="/test/gps_course_heading/output"),
        Parameter("debug_topic", value="/test/gps_course_heading/debug"),
    ])


def legacy_node() -> GpsCourseHeading:
    return GpsCourseHeading(parameter_overrides=[
        Parameter("drive_telemetry_topic", value=TEST_DRIVE_TOPIC),
        Parameter("rtk_status_topic", value=TEST_RTK_TOPIC),
    ])


def test_course_heading_evaluates_on_each_new_gps_fix() -> None:
    rclpy.init()
    node = legacy_node()
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


def test_valid_heading_waits_for_selector_discovery_without_exceeding_fix_freshness() -> None:
    rclpy.init()
    node = legacy_node()
    try:
        output = CapturingPublisher(subscription_count=0)
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
        node.on_fix(fix(2, 3.0))
        assert not output.messages
        assert node._pending_output is not None
        payload = json.loads(debug.messages[-1].data)
        assert payload["valid"] is True
        assert payload["pending_output"] is True
        assert payload["output_subscribers"] == 0

        output.subscription_count = 1
        node.now = lambda: 2.2
        node._flush_pending_output()
        assert len(output.messages) == 1
        assert node._pending_output is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_pending_heading_is_dropped_after_existing_fix_freshness_window() -> None:
    rclpy.init()
    node = legacy_node()
    try:
        output = CapturingPublisher(subscription_count=0)
        node.output = output
        node.debug = CapturingPublisher()
        node.speed = 1.2
        node.yaw_rate = 0.0
        node.steer = 0.0
        node.steer_valid = True
        node.rtk_status = "rtk_fixed"
        node.rtk_at_monotonic = time.monotonic()

        node.on_fix(fix(1, 0.0))
        node.on_fix(fix(2, 3.0))
        output.subscription_count = 1
        node.now = lambda: 3.0
        node._flush_pending_output()
        assert not output.messages
        assert node._pending_output is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_typed_wire_creates_only_the_canonical_rtk_subscription() -> None:
    rclpy.init()
    node = typed_node()
    try:
        subscriptions = node.get_subscriptions_info_by_topic(TEST_RTK_TOPIC)
        assert len(subscriptions) == 1
        assert subscriptions[0].topic_type == "salus_interfaces/msg/GnssRtkStatus"
        assert node._rtk_message_type is GnssRtkStatus
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_legacy_wire_remains_the_default_and_creates_only_string_subscription() -> None:
    rclpy.init()
    node = legacy_node()
    try:
        subscriptions = node.get_subscriptions_info_by_topic(TEST_RTK_TOPIC)
        assert len(subscriptions) == 1
        assert subscriptions[0].topic_type == "std_msgs/msg/String"
        assert node._rtk_message_type is String
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_typed_rtk_fixed_fresh_allows_course_heading() -> None:
    rclpy.init()
    node = typed_node()
    try:
        prepare_for_heading(node)
        node.on_gnss_rtk_status(typed_status(GnssRtkStatus.RTK_FIXED))
        node.on_fix(fix(1, 0.0))
        node.on_fix(fix(2, 3.0))
        payload = json.loads(node.debug.messages[-1].data)
        assert payload["valid"] is True
        assert payload["rtk_valid"] is True
        assert payload["reason"] == "ok"
        assert len(node.output.messages) == 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


@pytest.mark.parametrize("fix_quality", [
    GnssRtkStatus.RTK_FLOAT,
    GnssRtkStatus.DGPS,
    GnssRtkStatus.AUTONOMOUS,
    GnssRtkStatus.NO_FIX,
    GnssRtkStatus.UNKNOWN,
])
def test_typed_non_fixed_quality_rejects_even_with_fresh_corrections(
    fix_quality: int,
) -> None:
    rclpy.init()
    node = typed_node()
    try:
        prepare_for_heading(node)
        node.on_gnss_rtk_status(typed_status(fix_quality))
        node.on_fix(fix(1, 0.0))
        node.on_fix(fix(2, 3.0))
        payload = json.loads(node.debug.messages[-1].data)
        assert payload["valid"] is False
        assert payload["rtk_valid"] is False
        assert payload["reason"] == "rtk_status_rejected_or_stale"
        assert not node.output.messages
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_typed_rtk_fixed_stale_rejects_course_heading() -> None:
    rclpy.init()
    node = typed_node()
    try:
        prepare_for_heading(node)
        node.on_gnss_rtk_status(typed_status(GnssRtkStatus.RTK_FIXED))
        node.rtk_at_monotonic = time.monotonic() - 3.0
        node.on_rtk(String(data="rtk_fixed"))
        node.on_fix(fix(1, 0.0))
        node.on_fix(fix(2, 3.0))
        payload = json.loads(node.debug.messages[-1].data)
        assert payload["valid"] is False
        assert payload["rtk_valid"] is False
        assert payload["reason"] == "rtk_status_rejected_or_stale"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_typed_wire_does_not_fallback_to_a_later_legacy_string() -> None:
    rclpy.init()
    node = typed_node()
    try:
        prepare_for_heading(node)
        node.on_rtk(String(data="rtk_fixed"))
        node.on_fix(fix(1, 0.0))
        node.on_fix(fix(2, 3.0))
        payload = json.loads(node.debug.messages[-1].data)
        assert payload["valid"] is False
        assert payload["rtk_valid"] is False
        assert not node.output.messages
    finally:
        node.destroy_node()
        rclpy.shutdown()
