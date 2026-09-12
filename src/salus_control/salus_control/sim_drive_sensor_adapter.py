"""
Simulation-only drive, wheel and steering measurement adapter.

The adapter keeps Gazebo truth on its original topics and publishes delayed,
perturbed copies for the simulated control backend.  It never publishes TF or
commands, and all delivery scheduling is based on simulation timestamps.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import heapq
import math
import random
from typing import Any, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState


WHEEL_RNG_OFFSET = 101
STEERING_RNG_OFFSET = 211
DEFAULT_SIM_SENSOR_SEED = 6400
DEFAULT_MAX_PENDING_SAMPLES = 100
DEFAULT_RELEASE_TIMER_PERIOD_S = 0.001
DRIVE_ODOM_TOPIC = "/sim/sensors/drive_odom"
SIM_JOINT_STATES_TOPIC = "/sim/sensors/joint_states"
STEERING_JOINT_NAMES = (
    "front_left_steer_joint",
    "front_right_steer_joint",
)


@dataclass(frozen=True, slots=True)
class SimDriveSensorProfile:
    """
    Perturbation parameters for one simulation profile.

    Steering bias/noise are expressed in degrees at the configuration
    boundary, matching the issue contract; the processor converts them to
    radians before touching ``JointState.position``.
    """

    name: str
    wheel_scale: float
    wheel_bias_mps: float
    wheel_noise_sigma_mps: float
    wheel_dropout_probability: float
    wheel_latency_base_s: float
    wheel_jitter_sigma_s: float
    steering_bias_deg: float
    steering_noise_sigma_deg: float
    steering_dropout_probability: float
    steering_latency_base_s: float
    steering_jitter_sigma_s: float


SIM_DRIVE_SENSOR_PROFILES = {
    "clean": SimDriveSensorProfile(
        name="clean",
        wheel_scale=1.0,
        wheel_bias_mps=0.0,
        wheel_noise_sigma_mps=0.0,
        wheel_dropout_probability=0.0,
        wheel_latency_base_s=0.0,
        wheel_jitter_sigma_s=0.0,
        steering_bias_deg=0.0,
        steering_noise_sigma_deg=0.0,
        steering_dropout_probability=0.0,
        steering_latency_base_s=0.0,
        steering_jitter_sigma_s=0.0,
    ),
    "independent_nominal": SimDriveSensorProfile(
        name="independent_nominal",
        wheel_scale=1.005,
        wheel_bias_mps=0.0,
        wheel_noise_sigma_mps=0.015,
        wheel_dropout_probability=0.005,
        wheel_latency_base_s=0.020,
        wheel_jitter_sigma_s=0.005,
        steering_bias_deg=0.15,
        steering_noise_sigma_deg=0.20,
        steering_dropout_probability=0.005,
        steering_latency_base_s=0.020,
        steering_jitter_sigma_s=0.005,
    ),
    # The issue intentionally leaves degraded magnitudes open.  This preset
    # is a deterministic fault-characterization profile, not hardware
    # calibration: both streams have a reproducible outage and larger errors.
    "degraded": SimDriveSensorProfile(
        name="degraded",
        wheel_scale=1.02,
        wheel_bias_mps=0.0,
        wheel_noise_sigma_mps=0.08,
        wheel_dropout_probability=1.0,
        wheel_latency_base_s=0.25,
        wheel_jitter_sigma_s=0.05,
        steering_bias_deg=1.0,
        steering_noise_sigma_deg=1.0,
        steering_dropout_probability=1.0,
        steering_latency_base_s=0.25,
        steering_jitter_sigma_s=0.05,
    ),
}


def resolve_sim_drive_sensor_profile(name: str) -> SimDriveSensorProfile:
    """Return a named profile or fail closed for an unknown profile."""
    normalized = str(name).strip().lower()
    try:
        return SIM_DRIVE_SENSOR_PROFILES[normalized]
    except KeyError as exc:
        choices = ", ".join(sorted(SIM_DRIVE_SENSOR_PROFILES))
        raise ValueError(
            f"Unsupported sim_sensor_profile={name!r}; expected one of {choices}"
        ) from exc


def _validate_probability(value: float, field: str) -> float:
    result = float(value)
    if not math.isfinite(result) or not 0.0 <= result <= 1.0:
        raise ValueError(f"{field} must be finite and between 0 and 1")
    return result


def _validate_nonnegative(value: float, field: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result < 0.0:
        raise ValueError(f"{field} must be finite and non-negative")
    return result


def _stamp_seconds(message: Any) -> float:
    stamp = message.header.stamp
    result = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    if not math.isfinite(result) or result < 0.0:
        raise ValueError("measurement stamp must be finite and non-negative")
    return result


def _schedule_time(
    stamp_s: float, latency_s: float, jitter_s: float, rng: random.Random
) -> float:
    delay_s = max(0.0, float(latency_s) + rng.gauss(0.0, float(jitter_s)))
    return float(stamp_s) + delay_s


@dataclass(frozen=True, slots=True)
class ScheduledMeasurement:
    """A message and its simulation-time release deadline."""

    release_time_s: float
    message: Any


class SimDriveSensorProcessor:
    """Pure deterministic wheel/steering perturbation and scheduling policy."""

    def __init__(
        self,
        profile: SimDriveSensorProfile,
        seed: int = DEFAULT_SIM_SENSOR_SEED,
        steering_joint_names: tuple[str, ...] = STEERING_JOINT_NAMES,
    ) -> None:
        if not isinstance(profile, SimDriveSensorProfile):
            raise TypeError("profile must be a SimDriveSensorProfile")
        if not isinstance(seed, int) or isinstance(seed, bool):
            raise ValueError("sim_sensor_seed must be an integer")
        if not steering_joint_names or any(
            not str(name).strip() for name in steering_joint_names
        ):
            raise ValueError("steering_joint_names must contain non-empty names")
        self.profile = profile
        self.seed = seed
        self.steering_joint_names = frozenset(str(name) for name in steering_joint_names)
        self._validate_profile(profile)
        self._wheel_random = random.Random(seed + WHEEL_RNG_OFFSET)
        self._steering_random = random.Random(seed + STEERING_RNG_OFFSET)

    @staticmethod
    def _validate_profile(profile: SimDriveSensorProfile) -> None:
        finite_fields = (
            "wheel_scale",
            "wheel_bias_mps",
            "wheel_noise_sigma_mps",
            "wheel_latency_base_s",
            "wheel_jitter_sigma_s",
            "steering_bias_deg",
            "steering_noise_sigma_deg",
            "steering_latency_base_s",
            "steering_jitter_sigma_s",
        )
        for field in finite_fields:
            value = float(getattr(profile, field))
            if not math.isfinite(value):
                raise ValueError(f"{field} must be finite")
        if float(profile.wheel_scale) < 0.0:
            raise ValueError("wheel_scale must be non-negative")
        for field in (
            "wheel_noise_sigma_mps",
            "wheel_latency_base_s",
            "wheel_jitter_sigma_s",
            "steering_noise_sigma_deg",
            "steering_latency_base_s",
            "steering_jitter_sigma_s",
        ):
            _validate_nonnegative(getattr(profile, field), field)
        _validate_probability(
            profile.wheel_dropout_probability, "wheel_dropout_probability"
        )
        _validate_probability(
            profile.steering_dropout_probability,
            "steering_dropout_probability",
        )

    @staticmethod
    def _has_finite_steering_positions(
        message: JointState, names: frozenset[str]
    ) -> bool:
        for name, position in zip(message.name, message.position):
            if name in names and not math.isfinite(float(position)):
                return False
        return True

    def process_odom(self, message: Odometry) -> Optional[ScheduledMeasurement]:
        """Perturb wheel speed and return a delayed copy, or drop the sample."""
        stamp_s = _stamp_seconds(message)
        truth_speed_mps = float(message.twist.twist.linear.x)
        if not math.isfinite(truth_speed_mps):
            return None
        if self._wheel_random.random() < self.profile.wheel_dropout_probability:
            return None
        output = deepcopy(message)
        output.twist.twist.linear.x = (
            truth_speed_mps * self.profile.wheel_scale
            + self.profile.wheel_bias_mps
            + self._wheel_random.gauss(0.0, self.profile.wheel_noise_sigma_mps)
        )
        release_time_s = _schedule_time(
            stamp_s,
            self.profile.wheel_latency_base_s,
            self.profile.wheel_jitter_sigma_s,
            self._wheel_random,
        )
        return ScheduledMeasurement(release_time_s, output)

    def process_joint_states(
        self, message: JointState
    ) -> Optional[ScheduledMeasurement]:
        """Perturb steering positions and return a delayed copy, if valid."""
        stamp_s = _stamp_seconds(message)
        if not self._has_finite_steering_positions(message, self.steering_joint_names):
            return None
        if self._steering_random.random() < self.profile.steering_dropout_probability:
            return None
        output = deepcopy(message)
        steering_error_rad = math.radians(
            self.profile.steering_bias_deg
            + self._steering_random.gauss(0.0, self.profile.steering_noise_sigma_deg)
        )
        for index, name in enumerate(output.name):
            if name in self.steering_joint_names and index < len(output.position):
                output.position[index] = float(output.position[index]) + steering_error_rad
        release_time_s = _schedule_time(
            stamp_s,
            self.profile.steering_latency_base_s,
            self.profile.steering_jitter_sigma_s,
            self._steering_random,
        )
        return ScheduledMeasurement(release_time_s, output)


@dataclass(order=True, slots=True)
class _PendingMeasurement:
    release_time_s: float
    sequence: int
    message: Any


class _SimTimeQueue:
    """Bounded min-heap for delivery deadlines, reset on clock rollback."""

    def __init__(self, max_size: int) -> None:
        if int(max_size) <= 0:
            raise ValueError("max_pending_samples must be positive")
        self._max_size = int(max_size)
        self._items: list[_PendingMeasurement] = []
        self._sequence = 0

    def clear(self) -> None:
        self._items.clear()

    def push(self, scheduled: ScheduledMeasurement) -> None:
        item = _PendingMeasurement(
            float(scheduled.release_time_s), self._sequence, scheduled.message
        )
        self._sequence += 1
        heapq.heappush(self._items, item)
        if len(self._items) > self._max_size:
            heapq.heappop(self._items)

    def pop_ready(self, now_s: float) -> list[Any]:
        ready = []
        while self._items and self._items[0].release_time_s <= float(now_s):
            ready.append(heapq.heappop(self._items).message)
        return ready


class SimDriveSensorAdapterNode(Node):
    """ROS adapter for the simulation-only processor."""

    def __init__(self) -> None:
        super().__init__("sim_drive_sensor_adapter")
        self.declare_parameter("odom_topic", "/odom_raw")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("drive_odom_topic", DRIVE_ODOM_TOPIC)
        self.declare_parameter("sim_joint_states_topic", SIM_JOINT_STATES_TOPIC)
        self.declare_parameter("sim_sensor_profile", "clean")
        self.declare_parameter("sim_sensor_seed", DEFAULT_SIM_SENSOR_SEED)
        self.declare_parameter("max_pending_samples", DEFAULT_MAX_PENDING_SAMPLES)
        self.declare_parameter("release_timer_period_s", DEFAULT_RELEASE_TIMER_PERIOD_S)

        profile = resolve_sim_drive_sensor_profile(
            str(self.get_parameter("sim_sensor_profile").value)
        )
        seed = int(self.get_parameter("sim_sensor_seed").value)
        max_pending_samples = int(self.get_parameter("max_pending_samples").value)
        timer_period_s = _validate_nonnegative(
            self.get_parameter("release_timer_period_s").value,
            "release_timer_period_s",
        )
        if timer_period_s <= 0.0:
            raise ValueError("release_timer_period_s must be positive")
        self._processor = SimDriveSensorProcessor(profile, seed)
        self._odom_queue = _SimTimeQueue(max_pending_samples)
        self._joint_queue = _SimTimeQueue(max_pending_samples)
        self._last_clock_s: Optional[float] = None

        odom_topic = str(self.get_parameter("odom_topic").value)
        joint_states_topic = str(self.get_parameter("joint_states_topic").value)
        self._drive_odom_publisher = self.create_publisher(
            Odometry, str(self.get_parameter("drive_odom_topic").value), 10
        )
        self._joint_states_publisher = self.create_publisher(
            JointState, str(self.get_parameter("sim_joint_states_topic").value), 10
        )
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(
            JointState, joint_states_topic, self._on_joint_states, 10
        )
        self.create_timer(timer_period_s, self._release_ready)

    def _observe_clock(self, clock_s: float) -> None:
        if self._last_clock_s is not None and clock_s < self._last_clock_s:
            self._odom_queue.clear()
            self._joint_queue.clear()
        self._last_clock_s = clock_s

    def _sim_clock_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _on_odom(self, message: Odometry) -> None:
        self._observe_clock(self._sim_clock_seconds())
        try:
            scheduled = self._processor.process_odom(message)
        except ValueError as exc:
            self.get_logger().warning(f"Dropping invalid simulated odometry: {exc}")
            return
        if scheduled is not None:
            self._odom_queue.push(scheduled)

    def _on_joint_states(self, message: JointState) -> None:
        self._observe_clock(self._sim_clock_seconds())
        try:
            scheduled = self._processor.process_joint_states(message)
        except ValueError as exc:
            self.get_logger().warning(f"Dropping invalid simulated joint state: {exc}")
            return
        if scheduled is not None:
            self._joint_queue.push(scheduled)

    def _release_ready(self) -> None:
        now_s = self._sim_clock_seconds()
        if not math.isfinite(now_s) or now_s < 0.0:
            return
        self._observe_clock(now_s)
        for message in self._odom_queue.pop_ready(now_s):
            self._drive_odom_publisher.publish(message)
        for message in self._joint_queue.pop_ready(now_s):
            self._joint_states_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SimDriveSensorAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
