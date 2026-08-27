"""Explicit, fail-silent selection of one global orientation source."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String


COURSE_OVER_GROUND = "course_over_ground"
EXTERNAL_HEADING = "external_heading"
VALID_ORIENTATION_SOURCES = (COURSE_OVER_GROUND, EXTERNAL_HEADING)


@dataclass(frozen=True)
class OrientationDecision:
    accepted: bool
    reason: str


def normalize_orientation_source(value: object) -> str:
    source = str(value).strip().lower()
    if source not in VALID_ORIENTATION_SOURCES:
        raise ValueError(
            "orientation_source must be course_over_ground or external_heading"
        )
    return source


def stamp_nanoseconds(message: Imu) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


class OrientationSelectionPolicy:
    """Validate only the configured source; never fall back to another one."""

    def __init__(self, selected_source: object, expected_frame: object) -> None:
        self.selected_source = normalize_orientation_source(selected_source)
        self.expected_frame = str(expected_frame).strip()
        if not self.expected_frame:
            raise ValueError("expected orientation frame must not be empty")
        self._last_stamp_ns: int | None = None

    def evaluate(self, source_id: object, message: Imu) -> OrientationDecision:
        if str(source_id).strip().lower() != self.selected_source:
            return OrientationDecision(False, "source_not_selected")
        if message.header.frame_id != self.expected_frame:
            return OrientationDecision(False, "unexpected_frame")
        stamp_ns = stamp_nanoseconds(message)
        if stamp_ns <= 0:
            return OrientationDecision(False, "non_positive_timestamp")
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            return OrientationDecision(False, "non_monotonic_timestamp")
        quaternion = message.orientation
        values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
        if not all(math.isfinite(value) for value in values):
            return OrientationDecision(False, "non_finite_orientation")
        norm_squared = sum(value * value for value in values)
        if norm_squared <= 1.0e-9:
            return OrientationDecision(False, "degenerate_orientation")
        if abs(norm_squared - 1.0) > 1.0e-3:
            return OrientationDecision(False, "unnormalized_orientation")
        covariance = tuple(float(value) for value in message.orientation_covariance)
        if len(covariance) != 9 or not all(math.isfinite(value) for value in covariance):
            return OrientationDecision(False, "invalid_covariance")
        if covariance[0] < 0.0 or covariance[4] < 0.0 or covariance[8] <= 0.0:
            return OrientationDecision(False, "orientation_unavailable")
        self._last_stamp_ns = stamp_ns
        return OrientationDecision(True, "accepted")


class OrientationSourceSelectorNode(Node):
    """Subscribe to exactly one configured heading input and expose one output."""

    def __init__(self) -> None:
        super().__init__("orientation_source_selector")
        self.declare_parameter("selected_source", COURSE_OVER_GROUND)
        self.declare_parameter("course_topic", "/gps/course_heading")
        self.declare_parameter("external_topic", "/heading/external")
        self.declare_parameter("output_topic", "/localization/orientation")
        self.declare_parameter("expected_frame", "base_footprint")
        self.declare_parameter(
            "debug_topic", "/localization/orientation_selection/debug"
        )
        self._selected_source = normalize_orientation_source(
            self.get_parameter("selected_source").value
        )
        self._policy = OrientationSelectionPolicy(
            self._selected_source, self.get_parameter("expected_frame").value
        )
        input_parameter = (
            "course_topic"
            if self._selected_source == COURSE_OVER_GROUND
            else "external_topic"
        )
        self._publisher = self.create_publisher(
            Imu, str(self.get_parameter("output_topic").value), 10
        )
        self._debug_publisher = self.create_publisher(
            String, str(self.get_parameter("debug_topic").value), 10
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter(input_parameter).value),
            self._on_orientation,
            qos_profile_sensor_data,
        )
        self._publish_debug("configured")

    def _publish_debug(self, reason: str) -> None:
        self._debug_publisher.publish(
            String(
                data=json.dumps(
                    {"selected_source": self._selected_source, "reason": reason},
                    sort_keys=True,
                )
            )
        )

    def _on_orientation(self, message: Imu) -> None:
        decision = self._policy.evaluate(self._selected_source, message)
        self._publish_debug(decision.reason)
        if decision.accepted:
            self._publisher.publish(deepcopy(message))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OrientationSourceSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
