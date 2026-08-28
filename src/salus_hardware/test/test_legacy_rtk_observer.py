import json
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String, UInt8MultiArray

from salus_hardware.legacy_rtk_observer import LegacyRtkObserverNode
from salus_hardware.rtk_domain import crc24q


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _frame() -> bytes:
    body = b"\xd3\x00\x02\x3e\x00"
    return body + crc24q(body).to_bytes(3, "big")


def test_observer_keeps_fresh_corrections_separate_from_no_fix() -> None:
    rclpy.init()
    node = LegacyRtkObserverNode(parameter_overrides=[
        Parameter("delivery_backend", value="disabled"),
    ])
    try:
        status_pub = CapturingPublisher()
        rtcm_pub = CapturingPublisher()
        node._status_pub = status_pub
        node._rtcm_pub = rtcm_pub
        source = String()
        source.data = json.dumps({
            "status_sequence": 1,
            "active_source_id": "base",
            "connected": True,
            "receiving_rtcm": True,
            "rtcm_age_s": 0.1,
            "received_count": 3,
            "crc_errors": 0,
            "config_path": "/secret/path",
        })
        fix = String()
        fix.data = "gps_no_fix"
        frame = UInt8MultiArray()
        frame.data = list(_frame())
        node._on_status(source)
        node._on_fix(fix)
        node._on_rtcm(frame)
        node._publish_status()

        output = status_pub.messages[-1]
        assert output.fix_quality == output.NO_FIX
        assert output.acquisition_state == output.ACQUISITION_RECEIVING
        assert output.corrections_fresh is True
        assert output.delivery_backend == output.BACKEND_DISABLED
        assert output.delivery_state == output.DELIVERY_DISABLED
        assert output.source_id == "base"
        assert "/secret/path" not in output.status_detail
        assert len(rtcm_pub.messages) == 1
        assert bytes(rtcm_pub.messages[0].data) == _frame()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_observer_subscribes_to_exactly_one_configured_legacy_rtcm_type() -> None:
    source = (Path(__file__).parents[1] / "salus_hardware/legacy_rtk_observer.py").read_text()
    assert source.count("create_subscription(\n            UInt8MultiArray") == 1
    assert "rtcm_msgs" not in source
    assert "mavros_msgs" not in source


def test_console_entry_points_are_packaged() -> None:
    setup = (Path(__file__).parents[1] / "setup.py").read_text()
    assert "legacy_rtk_observer = salus_hardware.legacy_rtk_observer:main" in setup
    assert "rtcm_dry_run_sink = salus_hardware.rtcm_dry_run_sink:main" in setup
    assert "pixhawk_rtk_adapter = salus_hardware.pixhawk_rtk_adapter:main" in setup
