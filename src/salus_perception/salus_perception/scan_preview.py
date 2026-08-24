"""Lossy, diagnostic-only reduction of the canonical safety scan."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def reduce_scan_preview(
    message: LaserScan,
    *,
    beam_stride: int,
    crop_angle_min_rad: float,
    crop_angle_max_rad: float,
    output_range_max_m: float,
) -> Optional[LaserScan]:
    """Return a bounded diagnostic scan or ``None`` for invalid input.

    This deliberately does not repair malformed sensor data.  Consumers use
    absence of a preview as diagnostic information, while safety continues to
    use the untouched ``/scan_clean`` stream.
    """

    if not message.ranges or not math.isfinite(message.angle_increment) or message.angle_increment <= 1.0e-9:
        return None
    if not math.isfinite(message.angle_min) or not math.isfinite(message.angle_max):
        return None
    if not math.isfinite(output_range_max_m) or output_range_max_m <= 0.0:
        return None

    stride = max(1, int(beam_stride))
    start_angle = max(float(message.angle_min), float(crop_angle_min_rad))
    end_angle = min(float(message.angle_max), float(crop_angle_max_rad))
    if end_angle < start_angle:
        return None

    start_index = max(0, int(math.ceil((start_angle - message.angle_min) / message.angle_increment)))
    end_index = min(
        len(message.ranges) - 1,
        int(math.floor((end_angle - message.angle_min) / message.angle_increment)),
    )
    if end_index < start_index:
        return None
    indices = list(range(start_index, end_index + 1, stride))
    if not indices:
        return None

    preview = LaserScan()
    preview.header = message.header
    preview.angle_min = float(message.angle_min + indices[0] * message.angle_increment)
    preview.angle_max = float(message.angle_min + indices[-1] * message.angle_increment)
    preview.angle_increment = float(message.angle_increment * stride)
    preview.time_increment = float(message.time_increment * stride)
    preview.scan_time = message.scan_time
    preview.range_min = message.range_min
    preview.range_max = min(float(message.range_max), float(output_range_max_m))
    if not math.isfinite(preview.range_max) or preview.range_max <= 0.0:
        return None

    for index in indices:
        reading = float(message.ranges[index])
        preview.ranges.append(float("inf") if math.isfinite(reading) and reading > preview.range_max else reading)
    # Intensities are intentionally omitted: the WebSocket preview must remain
    # bounded and Cockpit does not consume them.
    return preview


class ScanPreviewNode(Node):
    """Rate-limited `/scan_clean` to `/scan_preview` diagnostic adapter."""

    def __init__(self) -> None:
        super().__init__("scan_preview")
        defaults = {
            "source_topic": "/scan_clean",
            "output_topic": "/scan_preview",
            "publish_hz": 2.0,
            "beam_stride": 4,
            "crop_angle_min_rad": -1.57079632679,
            "crop_angle_max_rad": 1.57079632679,
            "output_range_max_m": 12.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self._publish_period_s = 1.0 / max(0.2, float(value("publish_hz")))
        self._beam_stride = max(1, int(value("beam_stride")))
        self._crop_min = float(value("crop_angle_min_rad"))
        self._crop_max = float(value("crop_angle_max_rad"))
        self._range_max = max(0.5, float(value("output_range_max_m")))
        self._last_publish_s = float("-inf")
        self._publisher = self.create_publisher(
            LaserScan,
            str(value("output_topic")),
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )
        self.create_subscription(LaserScan, str(value("source_topic")), self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, message: LaserScan) -> None:
        now_s = self.get_clock().now().nanoseconds / 1.0e9
        if now_s - self._last_publish_s < self._publish_period_s:
            return
        preview = reduce_scan_preview(
            message,
            beam_stride=self._beam_stride,
            crop_angle_min_rad=self._crop_min,
            crop_angle_max_rad=self._crop_max,
            output_range_max_m=self._range_max,
        )
        if preview is None:
            return
        self._publisher.publish(preview)
        self._last_publish_s = now_s


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanPreviewNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
