"""Read-only ROS adapter from live MAVROS topics to identified hardware topics."""

from __future__ import annotations

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix

from .pixhawk_sensor_domain import validate_gnss, validate_imu


class PixhawkSensorAdapterNode(Node):
    """Expose Pixhawk IMU and GNSS without starting MAVROS or changing hardware."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "pixhawk_sensor_adapter", parameter_overrides=parameter_overrides
        )
        defaults = {
            "imu_input_topic": "/imu/data",
            "imu_output_topic": "/hardware/imu_primary/data",
            "imu_expected_frame": "base_link",
            "gnss_input_topic": "/global_position/raw/fix",
            "gnss_output_topic": "/hardware/gnss_primary/fix",
            "gnss_expected_frame": "base_link",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def parameter(name: str) -> str:
            return str(self.get_parameter(name).value).strip()

        values = {name: parameter(name) for name in defaults}
        if not all(values.values()):
            raise ValueError("Pixhawk sensor topics and expected frames must not be empty")
        self._imu_expected_frame = values["imu_expected_frame"]
        self._gnss_expected_frame = values["gnss_expected_frame"]
        self._imu_publisher = self.create_publisher(
            Imu, values["imu_output_topic"], qos_profile_sensor_data
        )
        self._gnss_publisher = self.create_publisher(
            NavSatFix, values["gnss_output_topic"], qos_profile_sensor_data
        )
        self._imu_subscription = self.create_subscription(
            Imu,
            values["imu_input_topic"],
            self._on_imu,
            qos_profile_sensor_data,
        )
        self._gnss_subscription = self.create_subscription(
            NavSatFix,
            values["gnss_input_topic"],
            self._on_gnss,
            qos_profile_sensor_data,
        )
        self._rejected = {"imu": {}, "gnss": {}}

    def _reject(self, sensor: str, reason: str) -> None:
        counts = self._rejected[sensor]
        counts[reason] = counts.get(reason, 0) + 1
        if counts[reason] == 1:
            self.get_logger().warning(
                f"rejected {sensor} sample: {reason}; further repeats are counted"
            )

    def _on_imu(self, message: Imu) -> None:
        reason = validate_imu(message, expected_frame=self._imu_expected_frame)
        if reason == "accepted":
            self._imu_publisher.publish(deepcopy(message))
        else:
            self._reject("imu", reason)

    def _on_gnss(self, message: NavSatFix) -> None:
        reason = validate_gnss(message, expected_frame=self._gnss_expected_frame)
        if reason == "accepted":
            self._gnss_publisher.publish(deepcopy(message))
        else:
            self._reject("gnss", reason)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PixhawkSensorAdapterNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
