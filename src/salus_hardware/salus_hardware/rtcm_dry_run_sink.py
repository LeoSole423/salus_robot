"""Non-delivering RTCM consumer used to verify the canonical boundary."""

from __future__ import annotations

import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from salus_interfaces.msg import RtcmFrame
from std_msgs.msg import String

from .rtk_domain import sequence_transition, validate_rtcm3_frame


class RtcmDryRunSinkNode(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("rtcm_dry_run_sink", parameter_overrides=parameter_overrides)
        self.declare_parameter("input_topic", "/salus/hardware/rtcm/corrections")
        self.declare_parameter("status_topic", "/salus/hardware/rtcm/dry_run_status")
        self.declare_parameter("status_period_s", 1.0)
        input_topic = str(self.get_parameter("input_topic").value).strip()
        status_topic = str(self.get_parameter("status_topic").value).strip()
        period = float(self.get_parameter("status_period_s").value)
        if not input_topic or not status_topic or not math.isfinite(period) or period <= 0.0:
            raise ValueError("dry-run sink topics and period must be valid")
        self._received = 0
        self._rejected = 0
        self._last_sequence = None
        self._last_source_id = None
        self._last_received_ns = None
        self._subscription = self.create_subscription(
            RtcmFrame, input_topic, self._on_frame, 10
        )
        self._publisher = self.create_publisher(String, status_topic, 10)
        self.create_timer(period, self._publish_status)

    def _on_frame(self, message: RtcmFrame) -> None:
        reason = validate_rtcm3_frame(bytes(message.data))
        source_id = str(message.source_id)
        if source_id != self._last_source_id:
            self._last_sequence = None
        transition = sequence_transition(self._last_sequence, int(message.sequence))
        if reason != "accepted" or transition in ("invalid", "duplicate", "reset"):
            self._rejected += 1
            if reason == "accepted" and transition == "reset":
                self._last_sequence = int(message.sequence)
                self._last_source_id = source_id
            return
        self._last_sequence = int(message.sequence)
        self._last_source_id = source_id
        self._last_received_ns = self.get_clock().now().nanoseconds
        self._received += 1

    def _publish_status(self) -> None:
        age_s = None
        if self._last_received_ns is not None:
            age_s = max(
                0.0,
                (self.get_clock().now().nanoseconds - self._last_received_ns) / 1.0e9,
            )
        output = String()
        output.data = json.dumps(
            {
                "received_count": self._received,
                "rejected_count": self._rejected,
                "age_s": age_s,
            },
            separators=(",", ":"),
        )
        self._publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RtcmDryRunSinkNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
