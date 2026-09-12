"""Pure, deterministic receiver models for simulated GNSS.

The receiver model owns noise, quality, dropout and delivery timing. Gazebo's
raw NavSatFix remains the measurement source and is never modified in place.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import heapq
import math
import random

from sensor_msgs.msg import NavSatFix, NavSatStatus


GPS_RNG_OFFSET = 401


@dataclass(frozen=True)
class SimGpsQuality:
    """The receiver state represented by one published fix."""

    name: str
    rtk_status: str
    navsat_status: int
    horizontal_covariance_m2: float
    vertical_covariance_m2: float


@dataclass(frozen=True)
class SimGpsProfile:
    """Simulation-only receiver parameters, expressed in SI units."""

    name: str
    noise_m: float
    vertical_noise_m: float
    rate_hz: float
    covariance_m2: float
    rtk_status: str
    navsat_status: int
    dropout_probability: float = 0.0
    latency_s: float = 0.0
    jitter_sigma_s: float = 0.0
    degraded_rtk_status: str = "RTK_FLOAT"
    degraded_navsat_status: int = NavSatStatus.STATUS_FIX
    degraded_covariance_m2: float = 9.0
    degraded_vertical_noise_m: float = 4.0
    quality_transition_period_s: float = 0.0
    quality_degraded_duration_s: float = 0.0
    forced_dropout_windows_s: tuple[tuple[float, float], ...] = ()

    def __post_init__(self) -> None:
        for field_name in (
            "noise_m",
            "vertical_noise_m",
            "rate_hz",
            "covariance_m2",
            "dropout_probability",
            "latency_s",
            "jitter_sigma_s",
            "degraded_covariance_m2",
            "degraded_vertical_noise_m",
            "quality_transition_period_s",
            "quality_degraded_duration_s",
        ):
            value = float(getattr(self, field_name))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{field_name} must be finite and non-negative")
        if self.dropout_probability > 1.0:
            raise ValueError("dropout_probability must be in [0, 1]")
        if self.quality_degraded_duration_s > self.quality_transition_period_s > 0.0:
            raise ValueError(
                "quality_degraded_duration_s cannot exceed the transition period"
            )
        for start_s, end_s in self.forced_dropout_windows_s:
            if not (math.isfinite(start_s) and math.isfinite(end_s)) or end_s < start_s:
                raise ValueError("forced dropout windows must be finite and ordered")

    def quality_at(self, stamp_s: float) -> SimGpsQuality:
        """Return the deterministic quality state at a measurement timestamp."""
        if not math.isfinite(stamp_s):
            raise ValueError("stamp_s must be finite")
        degraded = False
        if self.quality_transition_period_s > 0.0:
            phase_s = stamp_s % self.quality_transition_period_s
            degraded = phase_s >= (
                self.quality_transition_period_s - self.quality_degraded_duration_s
            )
        if degraded:
            return SimGpsQuality(
                "degraded",
                self.degraded_rtk_status,
                self.degraded_navsat_status,
                self.degraded_covariance_m2,
                max(0.0004, self.degraded_vertical_noise_m**2),
            )
        return SimGpsQuality(
            "rtk_fixed",
            self.rtk_status,
            self.navsat_status,
            self.covariance_m2,
            max(0.0004, self.vertical_noise_m**2),
        )

    def in_forced_dropout(self, stamp_s: float) -> bool:
        return any(
            start_s <= stamp_s < end_s
            for start_s, end_s in self.forced_dropout_windows_s
        )


_F9P_RTK = SimGpsProfile(
    "f9p_rtk", 0.02, 0.04, 10.0, 0.02**2, "RTK_FIXED", NavSatStatus.STATUS_GBAS_FIX
)


# Legacy receiver names remain available for existing simulation callers. The
# three #64 campaign profiles are constructed from ROS parameters instead.
LEGACY_PROFILES = {
    "ideal": SimGpsProfile(
        "ideal", 0.0, 0.0, 0.0, 0.01**2, "SIM_IDEAL", NavSatStatus.STATUS_FIX
    ),
    "f9p_rtk": _F9P_RTK,
    "m8n": SimGpsProfile(
        "m8n", 1.5, 2.5, 5.0, 1.5**2, "3D_FIX", NavSatStatus.STATUS_FIX
    ),
}


def sim_gps_profile_from_parameters(
    name: str, parameters: dict[str, object]
) -> SimGpsProfile:
    """Build a campaign receiver profile from its ROS YAML parameters."""
    starts = tuple(float(value) for value in parameters["gnss.forced_dropout_start_s"])
    ends = tuple(float(value) for value in parameters["gnss.forced_dropout_end_s"])
    if len(starts) != len(ends):
        raise ValueError("GNSS forced dropout start/end arrays must have equal length")
    return SimGpsProfile(
        str(name).strip().lower(),
        float(parameters["gnss.noise_stddev_m"]),
        float(parameters["gnss.vertical_noise_stddev_m"]),
        float(parameters["gnss.rate_hz"]),
        float(parameters["gnss.covariance_m2"]),
        str(parameters["gnss.rtk_status"]),
        int(parameters["gnss.navsat_status"]),
        dropout_probability=float(parameters["gnss.dropout_probability"]),
        latency_s=float(parameters["gnss.latency_s"]),
        jitter_sigma_s=float(parameters["gnss.jitter_s"]),
        degraded_rtk_status=str(parameters["gnss.degraded_rtk_status"]),
        degraded_navsat_status=int(parameters["gnss.degraded_navsat_status"]),
        degraded_covariance_m2=float(parameters["gnss.degraded_covariance_m2"]),
        degraded_vertical_noise_m=float(
            parameters["gnss.degraded_vertical_noise_m"]
        ),
        quality_transition_period_s=float(
            parameters["gnss.quality_transition_period_s"]
        ),
        quality_degraded_duration_s=float(
            parameters["gnss.quality_degraded_duration_s"]
        ),
        forced_dropout_windows_s=tuple(zip(starts, ends)),
    )


def resolve_gps_profile(name: str) -> SimGpsProfile:
    """Resolve a legacy receiver profile or fail closed."""
    try:
        return LEGACY_PROFILES[str(name).strip().lower()]
    except KeyError as exc:
        raise ValueError("Unsupported gps_profile: " + str(name)) from exc


def geodetic_offset(
    latitude_deg: float, longitude_deg: float, north_m: float, east_m: float
) -> tuple[float, float]:
    """Apply a local north/east displacement to a geodetic coordinate."""
    meters_per_degree = 111_320.0
    return (
        latitude_deg + north_m / meters_per_degree,
        longitude_deg
        + east_m
        / (meters_per_degree * max(1e-6, abs(math.cos(math.radians(latitude_deg))))),
    )


@dataclass(order=True)
class _PendingFix:
    release_time_s: float
    sequence: int
    message: NavSatFix


class SimGpsFixProcessor:
    """Apply a deterministic receiver profile without ROS or wall-clock I/O."""

    def __init__(self, profile: SimGpsProfile, seed: int = 0, max_pending: int = 256) -> None:
        if not isinstance(max_pending, int) or max_pending <= 0:
            raise ValueError("max_pending must be a positive integer")
        self.profile = profile
        self.seed = int(seed)
        self.stream_seed = self.seed + GPS_RNG_OFFSET
        self._noise_random = random.Random(self.stream_seed + 1)
        self._dropout_random = random.Random(self.stream_seed + 2)
        self._jitter_random = random.Random(self.stream_seed + 3)
        self._pending: list[_PendingFix] = []
        self._sequence = 0
        self._max_pending = max_pending
        self.last_publish_ns = -1
        self._last_clock_s: float | None = None

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def reset(self) -> None:
        """Discard delivery state at the boundary between simulation trials."""
        self._pending.clear()
        self.last_publish_ns = -1
        self._last_clock_s = None

    def _observe_clock(self, now_s: float) -> None:
        if not math.isfinite(now_s):
            raise ValueError("now_s must be finite")
        if self._last_clock_s is not None and now_s < self._last_clock_s:
            self.reset()
        self._last_clock_s = now_s

    @staticmethod
    def _stamp_s(msg: NavSatFix) -> float:
        return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

    @staticmethod
    def _valid_input(msg: NavSatFix) -> bool:
        return all(
            math.isfinite(float(value))
            for value in (msg.latitude, msg.longitude, msg.altitude)
        )

    def _should_drop(self, stamp_s: float) -> bool:
        return self.profile.in_forced_dropout(stamp_s) or (
            self._dropout_random.random() < self.profile.dropout_probability
        )

    def _make_fix(self, msg: NavSatFix, stamp_s: float) -> NavSatFix:
        output = deepcopy(msg)
        north = self._noise_random.gauss(0.0, self.profile.noise_m)
        east = self._noise_random.gauss(0.0, self.profile.noise_m)
        output.latitude, output.longitude = geodetic_offset(
            output.latitude, output.longitude, north, east
        )
        output.altitude += self._noise_random.gauss(0.0, self.profile.vertical_noise_m)
        quality = self.profile.quality_at(stamp_s)
        output.position_covariance = [
            quality.horizontal_covariance_m2,
            0.0,
            0.0,
            0.0,
            quality.horizontal_covariance_m2,
            0.0,
            0.0,
            0.0,
            quality.vertical_covariance_m2,
        ]
        output.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        output.status.status = quality.navsat_status
        return output

    def _schedule(self, output: NavSatFix, stamp_s: float) -> None:
        delay_s = max(
            0.0,
            self.profile.latency_s
            + self._jitter_random.gauss(0.0, self.profile.jitter_sigma_s),
        )
        if len(self._pending) < self._max_pending:
            heapq.heappush(
                self._pending,
                _PendingFix(stamp_s + delay_s, self._sequence, output),
            )
        self._sequence += 1

    def process(self, msg: NavSatFix, now_s: float | None = None) -> NavSatFix | None:
        """Ingest one raw fix; return it only when delivery is immediate.

        Delayed messages are available through :meth:`release`. The original
        measurement stamp is retained in all cases.
        """
        stamp_s = self._stamp_s(msg)
        if not self._valid_input(msg) or not math.isfinite(stamp_s):
            return None
        current_s = stamp_s if now_s is None else float(now_s)
        self._observe_clock(current_s)
        if self.profile.rate_hz and self.last_publish_ns >= 0:
            if stamp_s * 1e9 - self.last_publish_ns < 1e9 / self.profile.rate_hz:
                return None
        self.last_publish_ns = int(round(stamp_s * 1e9))
        if self._should_drop(stamp_s):
            return None
        self._schedule(self._make_fix(msg, stamp_s), stamp_s)
        if self._pending and self._pending[0].release_time_s <= current_s:
            return heapq.heappop(self._pending).message
        return None

    def release(self, now_s: float) -> list[NavSatFix]:
        """Release all samples whose delivery time has arrived in sim-time."""
        now_s = float(now_s)
        self._observe_clock(now_s)
        released: list[NavSatFix] = []
        while self._pending and self._pending[0].release_time_s <= now_s:
            released.append(heapq.heappop(self._pending).message)
        return released

    def quality_for(self, msg: NavSatFix) -> SimGpsQuality:
        return self.profile.quality_at(self._stamp_s(msg))
