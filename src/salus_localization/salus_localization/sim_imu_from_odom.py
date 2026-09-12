"""Generate a deterministic simulated IMU stream from Gazebo odometry.

This is deliberately a simulation adapter: it is not used with physical IMU
hardware and it publishes no TF. The pure processor below keeps measurement
stamps while delivery latency is driven by simulation time.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import heapq
import math
import random

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu


IMU_RNG_OFFSET = 307


def _double_array_parameter(node: Node, name: str) -> list[float]:
    """Read a possibly empty statically typed ROS double-array parameter."""
    default = Parameter(name, Parameter.Type.DOUBLE_ARRAY, [])
    return list(node.get_parameter_or(name, default).value)


@dataclass(frozen=True)
class SimImuProfile:
    """Simulation-only IMU gyro parameters in SI units."""

    name: str
    gyro_bias_rps: float = 0.0
    gyro_noise_sigma_rps: float = 0.0
    dropout_probability: float = 0.0
    latency_s: float = 0.0
    jitter_sigma_s: float = 0.0
    forced_dropout_windows_s: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        for field_name in (
            "gyro_bias_rps",
            "gyro_noise_sigma_rps",
            "dropout_probability",
            "latency_s",
            "jitter_sigma_s",
        ):
            value = float(getattr(self, field_name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field_name} must be finite and non-negative")
        if self.dropout_probability > 1.0:
            raise ValueError("dropout_probability must be in [0, 1]")
        for start_s, end_s in self.forced_dropout_windows_s:
            if not (math.isfinite(start_s) and math.isfinite(end_s)) or end_s < start_s:
                raise ValueError("forced dropout windows must be finite and ordered")

    def in_forced_dropout(self, stamp_s: float) -> bool:
        return any(
            start_s <= stamp_s < end_s
            for start_s, end_s in self.forced_dropout_windows_s
        )


def sim_imu_profile_from_parameters(
    name: str, parameters: dict[str, object]
) -> SimImuProfile:
    """Build an IMU profile from the ROS parameters loaded from its YAML file."""
    starts = tuple(float(value) for value in parameters["imu.forced_dropout_start_s"])
    ends = tuple(float(value) for value in parameters["imu.forced_dropout_end_s"])
    if len(starts) != len(ends):
        raise ValueError("IMU forced dropout start/end arrays must have equal length")
    return SimImuProfile(
        str(name).strip().lower(),
        gyro_bias_rps=float(parameters["imu.gyro_bias_rad_s"]),
        gyro_noise_sigma_rps=float(parameters["imu.gyro_noise_stddev_rad_s"]),
        dropout_probability=float(parameters["imu.dropout_probability"]),
        latency_s=float(parameters["imu.latency_s"]),
        jitter_sigma_s=float(parameters["imu.jitter_s"]),
        forced_dropout_windows_s=tuple(zip(starts, ends)),
    )


def planar_yaw_from_quaternion(quaternion) -> float:
    """Extract the planar yaw angle from a ROS quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


@dataclass(order=True)
class _PendingImu:
    release_time_s: float
    sequence: int
    message: Imu


class SimImuProcessor:
    """Apply an IMU profile without ROS or wall-clock I/O."""

    def __init__(self, profile: SimImuProfile, seed: int = 0, max_pending: int = 256) -> None:
        if not isinstance(max_pending, int) or max_pending <= 0:
            raise ValueError("max_pending must be a positive integer")
        self.profile = profile
        self.seed = int(seed)
        self.stream_seed = self.seed + IMU_RNG_OFFSET
        self._noise_random = random.Random(self.stream_seed + 1)
        self._dropout_random = random.Random(self.stream_seed + 2)
        self._jitter_random = random.Random(self.stream_seed + 3)
        self._pending: list[_PendingImu] = []
        self._sequence = 0
        self._max_pending = max_pending
        self._last_clock_s: float | None = None

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def reset(self) -> None:
        """Discard delivery state at the boundary between simulation trials."""
        self._pending.clear()
        self._last_clock_s = None

    def _observe_clock(self, now_s: float) -> None:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        if self._last_clock_s is not None and now_s < self._last_clock_s:
            self.reset()
        self._last_clock_s = now_s

    @staticmethod
    def _stamp_s(msg: Odometry) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    @staticmethod
    def _valid_input(msg: Odometry) -> bool:
        values = (
            msg.twist.twist.angular.z,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        return all(math.isfinite(float(value)) for value in values)

    def _make_imu(self, msg: Odometry, stamp_s: float) -> Imu:
        imu = Imu()
        imu.header = deepcopy(msg.header)
        imu.orientation = deepcopy(msg.pose.pose.orientation)
        imu.angular_velocity.z = (
            float(msg.twist.twist.angular.z)
            + self.profile.gyro_bias_rps
            + self._noise_random.gauss(0.0, self.profile.gyro_noise_sigma_rps)
        )
        # The current motion simulation is planar and does not model acceleration.
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0
        return imu

    def process(self, msg: Odometry, now_s: float | None = None) -> Imu | None:
        """Ingest one odometry sample; delayed samples are returned by release."""
        stamp_s = self._stamp_s(msg)
        current_s = stamp_s if now_s is None else float(now_s)
        self._observe_clock(current_s)
        if not self._valid_input(msg) or not math.isfinite(stamp_s):
            return None
        if self.profile.in_forced_dropout(stamp_s) or (
            self._dropout_random.random() < self.profile.dropout_probability
        ):
            return None
        delay_s = max(
            0.0,
            self.profile.latency_s
            + self._jitter_random.gauss(0.0, self.profile.jitter_sigma_s),
        )
        if len(self._pending) >= self._max_pending:
            return None
        heapq.heappush(
            self._pending,
            _PendingImu(
                stamp_s + delay_s,
                self._sequence,
                self._make_imu(msg, stamp_s),
            ),
        )
        self._sequence += 1
        if self._pending[0].release_time_s <= current_s:
            return heapq.heappop(self._pending).message
        return None

    def release(self, now_s: float) -> list[Imu]:
        """Release all samples whose delivery time has arrived in sim-time."""
        now_s = float(now_s)
        self._observe_clock(now_s)
        released: list[Imu] = []
        while self._pending and self._pending[0].release_time_s <= now_s:
            released.append(heapq.heappop(self._pending).message)
        return released


class SimImuFromOdomNode(Node):
    """Convert `/odom_raw` into a profile-controlled `/imu/data_raw` stream."""

    def __init__(self) -> None:
        super().__init__("sim_imu_from_odom")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("imu_topic", "/imu/data_raw")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("sim_sensor_profile", "clean")
        self.declare_parameter("imu_profile", "")
        self.declare_parameter("sim_sensor_seed", 6400)
        for parameter, default in (
            ("imu.gyro_bias_rad_s", 0.0),
            ("imu.gyro_noise_stddev_rad_s", 0.0),
            ("imu.latency_s", 0.0),
            ("imu.jitter_s", 0.0),
            ("imu.dropout_probability", 0.0),
            ("imu.forced_dropout_start_s", Parameter.Type.DOUBLE_ARRAY),
            ("imu.forced_dropout_end_s", Parameter.Type.DOUBLE_ARRAY),
        ):
            self.declare_parameter(parameter, default)
        # Kept as a compatibility alias for callers of the old adapter.
        self.declare_parameter("random_seed", -1)
        self.declare_parameter("max_pending", 256)
        self.declare_parameter("delivery_poll_period_s", 0.01)
        profile_name = str(self.get_parameter("imu_profile").value).strip()
        if not profile_name:
            profile_name = str(self.get_parameter("sim_sensor_profile").value)
        seed = int(self.get_parameter("sim_sensor_seed").value)
        legacy_seed = int(self.get_parameter("random_seed").value)
        if legacy_seed >= 0:
            seed = legacy_seed
        self._frame_id = str(self.get_parameter("frame_id").value)
        profile_parameters = {
            parameter: self.get_parameter(parameter).value
            for parameter in (
                "imu.gyro_bias_rad_s",
                "imu.gyro_noise_stddev_rad_s",
                "imu.latency_s",
                "imu.jitter_s",
                "imu.dropout_probability",
            )
        }
        profile_parameters.update(
            {
                "imu.forced_dropout_start_s": _double_array_parameter(
                    self, "imu.forced_dropout_start_s"
                ),
                "imu.forced_dropout_end_s": _double_array_parameter(
                    self, "imu.forced_dropout_end_s"
                ),
            }
        )
        self._processor = SimImuProcessor(
            sim_imu_profile_from_parameters(profile_name, profile_parameters),
            seed=seed,
            max_pending=int(self.get_parameter("max_pending").value),
        )
        self._publisher = self.create_publisher(
            Imu, str(self.get_parameter("imu_topic").value), 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            10,
        )
        self._release_timer = self.create_timer(
            float(self.get_parameter("delivery_poll_period_s").value),
            self._release_ready,
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish(self, message: Imu) -> None:
        message.header.frame_id = self._frame_id
        self._publisher.publish(message)

    def _release_ready(self) -> None:
        for message in self._processor.release(self._now_s()):
            self._publish(message)

    def _on_odom(self, msg: Odometry) -> None:
        now_s = self._now_s()
        output = self._processor.process(msg, now_s=now_s)
        if output is not None:
            self._publish(output)
        for message in self._processor.release(now_s):
            self._publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimImuFromOdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
