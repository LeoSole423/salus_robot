"""Normalize Gazebo NavSat messages with a deterministic receiver profile."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from .gps_profiles import SimGpsFixProcessor, resolve_gps_profile


class SimGpsNormalizer(Node):
    def __init__(self) -> None:
        super().__init__("sim_gps_normalizer")
        self.declare_parameter("input_topic", "/gps/fix_raw")
        self.declare_parameter("output_topic", "/gps/fix")
        self.declare_parameter("rtk_status_topic", "/gps/rtk_status")
        self.declare_parameter("frame_id", "gps_link")
        self.declare_parameter("gps_profile", "f9p_rtk")
        self.declare_parameter("sim_sensor_profile", "clean")
        self.declare_parameter("sim_sensor_seed", 6400)
        # Kept as a compatibility alias for callers of the old adapter.
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("max_pending", 256)
        self.declare_parameter("delivery_poll_period_s", 0.01)
        self.frame_id = str(self.get_parameter("frame_id").value)
        sensor_profile = str(self.get_parameter("sim_sensor_profile").value).strip()
        gps_profile = str(self.get_parameter("gps_profile").value).strip()
        profile_name = sensor_profile if sensor_profile != "clean" else gps_profile
        seed = int(self.get_parameter("sim_sensor_seed").value)
        legacy_seed = int(self.get_parameter("random_seed").value)
        if legacy_seed >= 0:
            seed = legacy_seed
        self.processor = SimGpsFixProcessor(
            resolve_gps_profile(profile_name),
            seed=seed,
            max_pending=int(self.get_parameter("max_pending").value),
        )
        self.publisher = self.create_publisher(
            NavSatFix, str(self.get_parameter("output_topic").value), 10
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("rtk_status_topic").value), 10
        )
        self.create_subscription(
            NavSatFix,
            str(self.get_parameter("input_topic").value),
            self.on_fix,
            qos_profile_sensor_data,
        )
        self._release_timer = self.create_timer(
            float(self.get_parameter("delivery_poll_period_s").value),
            self._release_ready,
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, output: NavSatFix) -> None:
        output.header.frame_id = self.frame_id
        self.publisher.publish(output)
        self.status_publisher.publish(
            String(data=self.processor.quality_for(output).rtk_status)
        )

    def _release_ready(self) -> None:
        for output in self.processor.release(self._now_s()):
            self._publish(output)

    def on_fix(self, message: NavSatFix) -> None:
        now_s = self._now_s()
        output = self.processor.process(message, now_s=now_s)
        if output is not None:
            self._publish(output)
        for delayed_output in self.processor.release(now_s):
            self._publish(delayed_output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimGpsNormalizer()
    try: rclpy.spin(node)
    finally: node.destroy_node(); rclpy.shutdown()
