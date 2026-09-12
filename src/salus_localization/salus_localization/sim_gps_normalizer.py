"""Normalize Gazebo NavSat messages with a deterministic receiver profile."""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from .gps_profiles import (
    SimGpsFixProcessor,
    resolve_gps_profile,
    sim_gps_profile_from_parameters,
)


def _double_array_parameter(node: Node, name: str) -> list[float]:
    """Read a possibly empty statically typed ROS double-array parameter."""
    default = Parameter(name, Parameter.Type.DOUBLE_ARRAY, [])
    return list(node.get_parameter_or(name, default).value)


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
        for parameter, default in (
            ("gnss.noise_stddev_m", 0.02),
            ("gnss.vertical_noise_stddev_m", 0.04),
            ("gnss.rate_hz", 10.0),
            ("gnss.covariance_m2", 0.02**2),
            ("gnss.rtk_status", "RTK_FIXED"),
            ("gnss.navsat_status", 2),
            ("gnss.latency_s", 0.0),
            ("gnss.jitter_s", 0.0),
            ("gnss.dropout_probability", 0.0),
            ("gnss.degraded_rtk_status", "RTK_FLOAT"),
            ("gnss.degraded_navsat_status", 0),
            ("gnss.degraded_covariance_m2", 1.0),
            ("gnss.degraded_vertical_noise_m", 1.5),
            ("gnss.quality_transition_period_s", 0.0),
            ("gnss.quality_degraded_duration_s", 0.0),
            ("gnss.forced_dropout_start_s", Parameter.Type.DOUBLE_ARRAY),
            ("gnss.forced_dropout_end_s", Parameter.Type.DOUBLE_ARRAY),
        ):
            self.declare_parameter(parameter, default)
        # Kept as a compatibility alias for callers of the old adapter.
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("max_pending", 256)
        self.declare_parameter("delivery_poll_period_s", 0.01)
        self.frame_id = str(self.get_parameter("frame_id").value)
        sensor_profile = str(self.get_parameter("sim_sensor_profile").value).strip()
        gps_profile = str(self.get_parameter("gps_profile").value).strip()
        profile_name = sensor_profile
        seed = int(self.get_parameter("sim_sensor_seed").value)
        legacy_seed = int(self.get_parameter("random_seed").value)
        if legacy_seed >= 0:
            seed = legacy_seed
        profile_parameters = {
            parameter: self.get_parameter(parameter).value
            for parameter in (
                "gnss.noise_stddev_m",
                "gnss.vertical_noise_stddev_m",
                "gnss.rate_hz",
                "gnss.covariance_m2",
                "gnss.rtk_status",
                "gnss.navsat_status",
                "gnss.latency_s",
                "gnss.jitter_s",
                "gnss.dropout_probability",
                "gnss.degraded_rtk_status",
                "gnss.degraded_navsat_status",
                "gnss.degraded_covariance_m2",
                "gnss.degraded_vertical_noise_m",
                "gnss.quality_transition_period_s",
                "gnss.quality_degraded_duration_s",
            )
        }
        profile_parameters.update(
            {
                "gnss.forced_dropout_start_s": _double_array_parameter(
                    self, "gnss.forced_dropout_start_s"
                ),
                "gnss.forced_dropout_end_s": _double_array_parameter(
                    self, "gnss.forced_dropout_end_s"
                ),
            }
        )
        if sensor_profile == "clean" and gps_profile not in {"", "f9p_rtk"}:
            profile = resolve_gps_profile(gps_profile)
        else:
            profile = sim_gps_profile_from_parameters(profile_name, profile_parameters)
        self.processor = SimGpsFixProcessor(
            profile, seed=seed, max_pending=int(self.get_parameter("max_pending").value)
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
