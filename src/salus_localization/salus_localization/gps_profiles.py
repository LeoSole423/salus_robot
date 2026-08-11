"""Deterministic GPS profiles used only by the simulation adapter."""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import math
import random

from sensor_msgs.msg import NavSatFix, NavSatStatus


@dataclass(frozen=True)
class SimGpsProfile:
    name: str
    noise_m: float
    vertical_noise_m: float
    rate_hz: float
    covariance_m2: float
    rtk_status: str
    navsat_status: int


PROFILES = {
    "ideal": SimGpsProfile("ideal", 0.0, 0.0, 0.0, 0.01**2, "SIM_IDEAL", NavSatStatus.STATUS_FIX),
    "f9p_rtk": SimGpsProfile("f9p_rtk", 0.02, 0.04, 10.0, 0.02**2, "RTK_FIXED", NavSatStatus.STATUS_GBAS_FIX),
    "m8n": SimGpsProfile("m8n", 1.5, 2.5, 5.0, 1.5**2, "3D_FIX", NavSatStatus.STATUS_FIX),
}


def resolve_gps_profile(name: str) -> SimGpsProfile:
    try:
        return PROFILES[str(name).strip().lower()]
    except KeyError as exc:
        raise ValueError("Unsupported gps_profile: " + str(name)) from exc


def geodetic_offset(latitude_deg: float, longitude_deg: float, north_m: float, east_m: float) -> tuple[float, float]:
    meters_per_degree = 111_320.0
    return (latitude_deg + north_m / meters_per_degree, longitude_deg + east_m / (meters_per_degree * max(1e-6, abs(math.cos(math.radians(latitude_deg))))))


class SimGpsFixProcessor:
    def __init__(self, profile: SimGpsProfile, seed: int = 0) -> None:
        self.profile = profile
        self.random = random.Random(seed)
        self.last_publish_ns = -1

    def process(self, msg: NavSatFix) -> NavSatFix | None:
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self.profile.rate_hz and self.last_publish_ns >= 0 and stamp_ns - self.last_publish_ns < int(1e9 / self.profile.rate_hz):
            return None
        self.last_publish_ns = stamp_ns
        output = deepcopy(msg)
        north = self.random.gauss(0.0, self.profile.noise_m)
        east = self.random.gauss(0.0, self.profile.noise_m)
        output.latitude, output.longitude = geodetic_offset(output.latitude, output.longitude, north, east)
        output.altitude += self.random.gauss(0.0, self.profile.vertical_noise_m)
        output.position_covariance = [self.profile.covariance_m2, 0.0, 0.0, 0.0, self.profile.covariance_m2, 0.0, 0.0, 0.0, max(0.0004, self.profile.vertical_noise_m**2)]
        output.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        output.status.status = self.profile.navsat_status
        return output
