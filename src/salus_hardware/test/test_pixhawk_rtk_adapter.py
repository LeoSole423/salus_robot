import rclpy
from mavros_msgs.msg import GPSRAW
from rclpy.parameter import Parameter
from salus_interfaces.msg import GnssRtkStatus, RtcmFrame

from salus_hardware.pixhawk_rtk_adapter import PixhawkRtkAdapterNode
from salus_hardware.rtk_domain import crc24q


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _frame(sequence=1):
    body = b"\xd3\x00\x02\x3e\x00"
    message = RtcmFrame()
    message.source_id = "base"
    message.sequence = sequence
    message.data = list(body + crc24q(body).to_bytes(3, "big"))
    return message


def test_default_configuration_has_no_mavros_output_publisher():
    rclpy.init()
    node = PixhawkRtkAdapterNode()
    try:
        assert node._mavros_pub is None
        assert node._backend == GnssRtkStatus.BACKEND_DISABLED
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_gpsraw_six_is_fixed_independently_of_correction_state():
    rclpy.init()
    node = PixhawkRtkAdapterNode(parameter_overrides=[
        Parameter("delivery_backend", value="pixhawk_mavros"),
        Parameter("delivery_enabled", value=False),
    ])
    try:
        status_pub = CapturingPublisher()
        node._status_pub = status_pub
        gps = GPSRAW()
        gps.fix_type = GPSRAW.GPS_FIX_TYPE_RTK_FIXED
        gps.satellites_visible = 27
        node._on_gpsraw(gps)
        node._publish_status()
        status = status_pub.messages[-1]
        assert status.receiver_fix_type == 6
        assert status.fix_quality == status.RTK_FIXED
        assert status.satellites_visible == 27
        assert status.corrections_fresh is False
        assert status.delivery_state == status.DELIVERY_IDLE
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_delivery_requires_both_guards_and_rejects_duplicates():
    rclpy.init()
    node = PixhawkRtkAdapterNode(parameter_overrides=[
        Parameter("delivery_backend", value="pixhawk_mavros"),
        Parameter("delivery_enabled", value=True),
    ])
    try:
        output = CapturingPublisher()
        node._mavros_pub = output
        node._on_rtcm(_frame(1))
        node._on_rtcm(_frame(1))
        assert len(output.messages) == 1
        assert bytes(output.messages[0].data) == bytes(_frame(1).data)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_delivery_enabled_with_disabled_or_direct_usb_fails_closed():
    rclpy.init()
    try:
        for backend, enabled in (("disabled", True), ("direct_usb", False)):
            try:
                node = PixhawkRtkAdapterNode(parameter_overrides=[
                    Parameter("delivery_backend", value=backend),
                    Parameter("delivery_enabled", value=enabled),
                ])
            except ValueError:
                continue
            node.destroy_node()
            raise AssertionError(f"unsafe profile unexpectedly started: {backend}")
    finally:
        rclpy.shutdown()
