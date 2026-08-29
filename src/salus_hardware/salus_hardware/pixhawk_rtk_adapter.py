"""Explicit, fail-closed bridge from canonical RTCM to MAVROS."""

from __future__ import annotations

import math

import rclpy
from mavros_msgs.msg import GPSRAW, RTCM
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from salus_interfaces.msg import GnssRtkStatus, RtcmFrame

from .rtk_domain import (
    DeliveryBackend,
    DeliveryState,
    FixQuality,
    delivery_backend,
    evaluate_rtcm_delivery,
)


class PixhawkRtkAdapterNode(Node):
    """Observe GPSRAW and optionally deliver validated frames to MAVROS."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("pixhawk_rtk_adapter", parameter_overrides=parameter_overrides)
        defaults = {
            "source_status_topic": "/salus/hardware/gnss_primary/rtk_source_status",
            "rtcm_input_topic": "/salus/hardware/rtcm/corrections",
            "gpsraw_topic": "/mavros_node/gps1/raw",
            "status_topic": "/salus/hardware/gnss_primary/rtk_status",
            "mavros_rtcm_topic": "/mavros_node/send_rtcm",
            "delivery_backend": "disabled",
            "delivery_enabled": False,
            "stale_timeout_s": 5.0,
            "status_period_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def text(name: str) -> str:
            return str(self.get_parameter(name).value).strip()

        self._backend = delivery_backend(text("delivery_backend"))
        self._delivery_enabled = bool(self.get_parameter("delivery_enabled").value)
        self._stale_timeout_s = float(self.get_parameter("stale_timeout_s").value)
        period = float(self.get_parameter("status_period_s").value)
        if not math.isfinite(self._stale_timeout_s) or self._stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be positive and finite")
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError("status_period_s must be positive and finite")
        if self._backend == DeliveryBackend.DIRECT_USB:
            raise ValueError("direct_usb delivery backend is not implemented")
        if self._delivery_enabled and self._backend != DeliveryBackend.PIXHAWK_MAVROS:
            raise ValueError("delivery_enabled requires delivery_backend=pixhawk_mavros")

        self._source_status = None
        self._source_status_received_ns = None
        self._fix_quality = FixQuality.UNKNOWN
        self._receiver_fix_type = -1
        self._satellites_visible = 255
        self._gps_received_ns = None
        self._last_source_id = None
        self._last_sequence = None
        self._last_delivered_ns = None
        self._delivery_error = False

        self._status_pub = self.create_publisher(GnssRtkStatus, text("status_topic"), 10)
        # Fail closed: a MAVROS publisher does not exist unless both guards pass.
        self._mavros_pub = None
        if self._delivery_enabled:
            self._mavros_pub = self.create_publisher(RTCM, text("mavros_rtcm_topic"), 10)
        self.create_subscription(
            GnssRtkStatus, text("source_status_topic"), self._on_source_status, 10
        )
        self.create_subscription(RtcmFrame, text("rtcm_input_topic"), self._on_rtcm, 10)
        self.create_subscription(GPSRAW, text("gpsraw_topic"), self._on_gpsraw, 10)
        self.create_timer(period, self._publish_status)

    def _on_source_status(self, message: GnssRtkStatus) -> None:
        self._source_status = message
        self._source_status_received_ns = self.get_clock().now().nanoseconds

    def _on_gpsraw(self, message: GPSRAW) -> None:
        fix_type = int(message.fix_type)
        if fix_type == GPSRAW.GPS_FIX_TYPE_RTK_FLOAT:
            quality = FixQuality.RTK_FLOAT
        elif fix_type == GPSRAW.GPS_FIX_TYPE_RTK_FIXED:
            quality = FixQuality.RTK_FIXED
        elif fix_type == GPSRAW.GPS_FIX_TYPE_DGPS:
            quality = FixQuality.DGPS
        elif fix_type in (
            GPSRAW.GPS_FIX_TYPE_2D_FIX,
            GPSRAW.GPS_FIX_TYPE_3D_FIX,
            GPSRAW.GPS_FIX_TYPE_STATIC,
            GPSRAW.GPS_FIX_TYPE_PPP,
        ):
            quality = FixQuality.AUTONOMOUS
        elif fix_type in (GPSRAW.GPS_FIX_TYPE_NO_GPS, GPSRAW.GPS_FIX_TYPE_NO_FIX):
            quality = FixQuality.NO_FIX
        else:
            quality = FixQuality.UNKNOWN
        self._fix_quality = quality
        self._receiver_fix_type = fix_type
        satellites = int(message.satellites_visible)
        self._satellites_visible = min(255, max(0, satellites))
        self._gps_received_ns = self.get_clock().now().nanoseconds

    def _on_rtcm(self, message: RtcmFrame) -> None:
        decision = evaluate_rtcm_delivery(
            data=bytes(message.data),
            source_id=str(message.source_id),
            sequence=int(message.sequence),
            previous_source_id=self._last_source_id,
            previous_sequence=self._last_sequence,
        )
        self._last_source_id = decision.source_id
        self._last_sequence = decision.sequence
        if not decision.accepted or self._mavros_pub is None:
            return
        try:
            output = RTCM()
            output.header = message.header
            output.data = list(message.data)
            self._mavros_pub.publish(output)
            self._last_delivered_ns = self.get_clock().now().nanoseconds
            self._delivery_error = False
        except Exception:  # ROS middleware failures are surfaced without payload data.
            self._delivery_error = True
            self.get_logger().error("failed to deliver validated RTCM frame to MAVROS")

    def _publish_status(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        source = self._source_status
        output = GnssRtkStatus()
        output.header.stamp = self.get_clock().now().to_msg()
        gps_fresh = (
            self._gps_received_ns is not None
            and (now_ns - self._gps_received_ns) / 1.0e9 <= self._stale_timeout_s
        )
        output.fix_quality = int(self._fix_quality if gps_fresh else FixQuality.UNKNOWN)
        output.receiver_fix_type = self._receiver_fix_type if gps_fresh else -1
        output.satellites_visible = self._satellites_visible if gps_fresh else 255
        output.delivery_backend = int(self._backend)
        if self._backend == DeliveryBackend.DISABLED:
            delivery_state = DeliveryState.DISABLED
        elif not self._delivery_enabled:
            delivery_state = DeliveryState.IDLE
        elif self._delivery_error:
            delivery_state = DeliveryState.ERROR
        elif self._last_delivered_ns is None:
            delivery_state = DeliveryState.IDLE
        elif (now_ns - self._last_delivered_ns) / 1.0e9 > self._stale_timeout_s:
            delivery_state = DeliveryState.STALE
        else:
            delivery_state = DeliveryState.DELIVERING
        output.delivery_state = int(delivery_state)
        if source is not None:
            elapsed_s = max(
                0.0, (now_ns - self._source_status_received_ns) / 1.0e9
            )
            correction_age_s = float(source.correction_age_s)
            if math.isfinite(correction_age_s):
                correction_age_s += elapsed_s
            source_fresh = elapsed_s <= self._stale_timeout_s
            corrections_fresh = (
                source_fresh
                and bool(source.corrections_fresh)
                and math.isfinite(correction_age_s)
                and correction_age_s <= self._stale_timeout_s
            )
            output.acquisition_state = (
                source.acquisition_state
                if source_fresh
                else GnssRtkStatus.ACQUISITION_STALE
            )
            output.corrections_fresh = corrections_fresh
            output.correction_age_s = correction_age_s
            output.received_count = source.received_count
            output.crc_error_count = source.crc_error_count
            output.source_id = source.source_id
        else:
            output.acquisition_state = GnssRtkStatus.ACQUISITION_DISCONNECTED
            output.corrections_fresh = False
            output.correction_age_s = math.nan
        output.status_detail = "pixhawk_mavros_observation"
        self._status_pub.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PixhawkRtkAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
