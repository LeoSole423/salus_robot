"""ROS adapter that republishes one explicitly selected IMU source."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .imu_selection_policy import ImuSelectionConfig, ImuSelectionState, select_imu


class ImuSelectorNode(Node):
    """Publish valid samples from exactly one configured physical IMU source."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("imu_selector", parameter_overrides=parameter_overrides)
        defaults = {
            "selected_source": "imu_primary",
            "primary_topic": "/hardware/imu_primary/data",
            "secondary_topic": "/hardware/imu_secondary/data",
            "output_topic": "/imu/data",
            "primary_frame": "imu_primary_link",
            "secondary_frame": "imu_secondary_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameter = lambda name: self.get_parameter(name).value
        self._config = ImuSelectionConfig.create(
            selected_source=parameter("selected_source"),
            primary_frame=parameter("primary_frame"),
            secondary_frame=parameter("secondary_frame"),
        )
        selected_topic = str(
            parameter("primary_topic")
            if self._config.selected_source == "imu_primary"
            else parameter("secondary_topic")
        )
        if not selected_topic.strip():
            raise ValueError("selected IMU topic must not be empty")
        output_topic = str(parameter("output_topic"))
        if not output_topic.strip():
            raise ValueError("output_topic must not be empty")
        self._state = ImuSelectionState()
        # Preserve the established logical `/imu/data` reliable QoS contract.
        self._publisher = self.create_publisher(Imu, output_topic, 10)
        self._subscription = self.create_subscription(
            Imu, selected_topic, self._on_imu, qos_profile_sensor_data
        )

    def _on_imu(self, message: Imu) -> None:
        decision = select_imu(
            self._state,
            source_id=self._config.selected_source,
            message=message,
            config=self._config,
        )
        self._state = decision.state
        if decision.accepted:
            self._publisher.publish(deepcopy(message))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuSelectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
