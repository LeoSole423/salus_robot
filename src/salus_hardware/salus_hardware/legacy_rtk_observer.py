"""Read-only compatibility adapter for the deployed legacy RTK topics."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from salus_interfaces.msg import GnssRtkStatus, RtcmFrame
from std_msgs.msg import String, UInt8MultiArray

from .rtk_domain import (
    AcquisitionState,
    DeliveryBackend,
    DeliveryState,
    FixQuality,
    acquisition_state,
    age_source_status,
    delivery_backend,
    map_legacy_fix_status,
    parse_legacy_source_status,
    sequence_transition,
    validate_rtcm3_frame,
)


class LegacyRtkObserverNode(Node):
    """Normalize legacy observations without opening NTRIP or delivering RTCM."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("legacy_rtk_observer", parameter_overrides=parameter_overrides)
        defaults = {
            "legacy_status_topic": "/gps/rtk_source/status_json",
            "legacy_fix_topic": "/gps/rtk_status_mavros",
            "legacy_rtcm_topic": "/rtcm",
            "canonical_status_topic": "/salus/hardware/gnss_primary/rtk_status",
            "canonical_rtcm_topic": "/salus/hardware/rtcm/corrections",
            "legacy_rtcm_type": "uint8_multi_array",
            "delivery_backend": "disabled",
            "stale_timeout_s": 5.0,
            "status_period_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def topic(name: str) -> str:
            return str(self.get_parameter(name).value).strip()

        rtcm_type = topic("legacy_rtcm_type")
        if rtcm_type != "uint8_multi_array":
            raise ValueError("legacy_rtcm_type must be exactly uint8_multi_array")
        self._backend = delivery_backend(topic("delivery_backend"))
        self._stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        self._status_period_s = float(self.get_parameter("status_period_s").value)
        if self._stale_timeout_s <= 0.0 or self._status_period_s <= 0.0:
            raise ValueError("RTK observation periods must be positive")

        self._source_status = None
        self._source_status_received_ns = None
        self._source_sequence = None
        self._fix_quality = FixQuality.UNKNOWN
        self._receiver_fix_type = -1
        self._fix_received_ns = None
        self._frame_sequence = 0
        self._rejected_frames = 0
        self._crc_rejected_frames = 0

        self._status_pub = self.create_publisher(
            GnssRtkStatus, topic("canonical_status_topic"), 10
        )
        self._rtcm_pub = self.create_publisher(RtcmFrame, topic("canonical_rtcm_topic"), 10)
        self.create_subscription(String, topic("legacy_status_topic"), self._on_status, 10)
        self.create_subscription(String, topic("legacy_fix_topic"), self._on_fix, 10)
        # Deliberately exactly one legacy ROS type on /rtcm.
        self.create_subscription(
            UInt8MultiArray, topic("legacy_rtcm_topic"), self._on_rtcm, 10
        )
        self.create_timer(self._status_period_s, self._publish_status)

    def _on_status(self, message: String) -> None:
        try:
            candidate = parse_legacy_source_status(
                message.data, stale_timeout_s=self._stale_timeout_s
            )
        except ValueError:
            self.get_logger().warning("rejected legacy RTK status JSON")
            return
        transition = sequence_transition(self._source_sequence, candidate.sequence)
        self._source_sequence = candidate.sequence
        self._source_status = candidate
        self._source_status_received_ns = self.get_clock().now().nanoseconds
        if transition == "reset":
            self.get_logger().info("legacy RTK status sequence restarted")

    def _on_fix(self, message: String) -> None:
        self._fix_quality, self._receiver_fix_type = map_legacy_fix_status(message.data)
        self._fix_received_ns = self.get_clock().now().nanoseconds

    def _on_rtcm(self, message: UInt8MultiArray) -> None:
        data = bytes(message.data)
        reason = validate_rtcm3_frame(data)
        if reason != "accepted":
            self._rejected_frames += 1
            if reason == "crc_mismatch":
                self._crc_rejected_frames += 1
            if self._rejected_frames == 1:
                self.get_logger().warning(
                    f"rejected legacy RTCM frame: {reason}; repeats are counted"
                )
            return
        self._frame_sequence += 1
        output = RtcmFrame()
        output.header.stamp = self.get_clock().now().to_msg()
        output.source_id = self._source_status.source_id if self._source_status else ""
        output.sequence = self._frame_sequence
        output.data = list(data)
        self._rtcm_pub.publish(output)

    def _publish_status(self) -> None:
        elapsed_s = 0.0
        if self._source_status_received_ns is not None:
            elapsed_s = max(
                0.0,
                (self.get_clock().now().nanoseconds - self._source_status_received_ns)
                / 1.0e9,
            )
        source = age_source_status(self._source_status, elapsed_s=elapsed_s)
        state = acquisition_state(
            source, stale_timeout_s=self._stale_timeout_s
        )
        output = GnssRtkStatus()
        output.header.stamp = self.get_clock().now().to_msg()
        fix_fresh = (
            self._fix_received_ns is not None
            and (self.get_clock().now().nanoseconds - self._fix_received_ns) / 1.0e9
            <= self._stale_timeout_s
        )
        output.fix_quality = int(
            self._fix_quality if fix_fresh else FixQuality.UNKNOWN
        )
        output.acquisition_state = int(state)
        output.delivery_backend = int(self._backend)
        output.delivery_state = int(
            DeliveryState.DISABLED
            if self._backend == DeliveryBackend.DISABLED
            else DeliveryState.IDLE
        )
        output.receiver_fix_type = self._receiver_fix_type if fix_fresh else -1
        output.satellites_visible = 255
        output.corrections_fresh = state == AcquisitionState.RECEIVING
        output.correction_age_s = (
            float(source.correction_age_s)
            if source and math.isfinite(source.correction_age_s)
            else math.nan
        )
        output.received_count = source.received_count if source else 0
        output.crc_error_count = (
            (source.crc_error_count if source else 0) + self._crc_rejected_frames
        )
        output.source_id = source.source_id if source else ""
        if state == AcquisitionState.STALE:
            output.status_detail = "corrections_stale"
        else:
            output.status_detail = source.detail if source else "waiting_for_legacy_status"
        self._status_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LegacyRtkObserverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
