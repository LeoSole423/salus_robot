"""ROS adapter that republishes one explicitly selected physical GNSS source."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix

from .gnss_selection_policy import GnssSelectionPolicy


class GnssSelectorNode(Node):
    """Expose a single logical GNSS topic and never fall back implicitly."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("gnss_selector", parameter_overrides=parameter_overrides)
        defaults = {
            "selected_source": "gnss_primary",
            "primary_topic": "/hardware/gnss_primary/fix",
            "secondary_topic": "/hardware/gnss_secondary/fix",
            "output_topic": "/gps/fix",
            "primary_frame": "base_link",
            "secondary_frame": "gnss_secondary_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def parameter(name: str) -> str:
            return str(self.get_parameter(name).value).strip()

        selected = parameter("selected_source").lower()
        expected_frame = parameter(
            "primary_frame" if selected == "gnss_primary" else "secondary_frame"
        )
        self._policy = GnssSelectionPolicy(selected, expected_frame)
        input_topic = parameter(
            "primary_topic" if selected == "gnss_primary" else "secondary_topic"
        )
        output_topic = parameter("output_topic")
        if not input_topic or not output_topic:
            raise ValueError("selected GNSS input and output topics must not be empty")
        self._publisher = self.create_publisher(NavSatFix, output_topic, 10)
        self._subscription = self.create_subscription(
            NavSatFix, input_topic, self._on_fix, qos_profile_sensor_data
        )

    def _on_fix(self, message: NavSatFix) -> None:
        decision = self._policy.evaluate(self._policy.selected_source, message)
        if decision.accepted:
            self._publisher.publish(deepcopy(message))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GnssSelectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
