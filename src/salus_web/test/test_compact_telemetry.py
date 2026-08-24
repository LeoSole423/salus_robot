import math

import pytest
from sensor_msgs.msg import LaserScan

from salus_web.compact_telemetry import CompactTelemetryPolicy, normalize_telemetry_profile
from salus_web.ros_gateway import scan_preview_payload
from salus_web.websocket_server import REPLACEABLE_OPS


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now


def test_compact_policy_emits_first_and_transitions_but_coalesces_measurements() -> None:
    clock = Clock()
    policy = CompactTelemetryPolicy(2.0, clock)
    cache = {"goal_active": False, "robot_pose": {"lat": -31.0}}
    assert policy.observe(cache) is True
    cache["robot_pose"] = {"lat": -31.1}
    assert policy.observe(cache) is False
    assert policy.due() is False
    clock.now = 0.5
    assert policy.due() is True
    policy.mark_emitted()
    cache["goal_active"] = True
    assert policy.observe(cache) is True


@pytest.mark.parametrize("value", ["", "wifi", "debug", None])
def test_telemetry_profile_rejects_ambiguous_values(value) -> None:
    with pytest.raises(ValueError):
        normalize_telemetry_profile(value)
    assert normalize_telemetry_profile("COMPACT") == "compact"
    assert normalize_telemetry_profile("full") == "full"


def test_scan_preview_projection_is_bounded_and_leaves_nonfinite_json_safe() -> None:
    message = LaserScan()
    message.header.frame_id = "base_footprint"
    message.header.stamp.sec = 12
    message.angle_min = -1.0
    message.angle_increment = 0.1
    message.range_min = 0.4
    message.range_max = 12.0
    message.ranges = [1.0, float("inf"), math.nan, 13.0]
    payload = scan_preview_payload(message)
    assert payload is not None
    assert payload["op"] == "scan_preview"
    assert payload["stamp"] == {"sec": 12, "nanosec": 0}
    assert payload["valid_count"] == 1
    assert payload["ranges"][0] == 1.0
    assert math.isinf(payload["ranges"][1])
    assert "scan_preview" in REPLACEABLE_OPS


def test_scan_preview_projection_rejects_invalid_structural_input() -> None:
    message = LaserScan()
    message.ranges = [1.0]
    message.angle_increment = 0.1
    message.range_min = 0.4
    message.range_max = 12.0
    assert scan_preview_payload(message) is None
