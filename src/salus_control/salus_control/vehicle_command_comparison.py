"""Pure comparison policy for the legacy vehicle-command shadow."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .legacy_vehicle_command import VehicleCommandValue


@dataclass(frozen=True, slots=True)
class ObservedVehicleCommand:
    source: int
    drive_enabled: bool
    emergency_stop: bool
    brake_ratio: float
    speed_mps: float
    steering_angle_rad: float
    valid_for_s: float
    frame_id: str
    stamp_ns: int


@dataclass(frozen=True, slots=True)
class ComparisonTolerances:
    speed_mps: float = 1.0e-5
    steering_angle_rad: float = 1.0e-5
    brake_ratio: float = 1.0e-5
    valid_for_s: float = 1.0e-6
    expected_frame_id: str = "base_footprint"

    def __post_init__(self) -> None:
        values = (
            self.speed_mps,
            self.steering_angle_rad,
            self.brake_ratio,
            self.valid_for_s,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError("comparison tolerances must be finite and nonnegative")
        if not self.expected_frame_id.strip():
            raise ValueError("expected_frame_id must not be empty")


@dataclass(frozen=True, slots=True)
class ComparisonResult:
    matches: bool
    reasons: tuple[str, ...]


def compare_vehicle_commands(
    expected: VehicleCommandValue,
    observed: ObservedVehicleCommand,
    tolerances: ComparisonTolerances,
) -> ComparisonResult:
    """Compare canonical semantics without treating shadow as authoritative."""
    reasons: list[str] = []
    scalar_fields = (
        (
            "brake_ratio",
            expected.brake_ratio,
            observed.brake_ratio,
            tolerances.brake_ratio,
        ),
        ("speed_mps", expected.speed_mps, observed.speed_mps, tolerances.speed_mps),
        (
            "steering_angle_rad",
            expected.steering_angle_rad,
            observed.steering_angle_rad,
            tolerances.steering_angle_rad,
        ),
        (
            "valid_for_s",
            expected.valid_for_s,
            observed.valid_for_s,
            tolerances.valid_for_s,
        ),
    )
    for name, expected_value, observed_value, tolerance in scalar_fields:
        if not math.isfinite(observed_value):
            reasons.append(f"{name}_nonfinite")
        elif abs(expected_value - observed_value) > tolerance:
            reasons.append(f"{name}_mismatch")
    for name, expected_value, observed_value in (
        ("source", expected.source, observed.source),
        ("drive_enabled", expected.drive_enabled, observed.drive_enabled),
        ("emergency_stop", expected.emergency_stop, observed.emergency_stop),
    ):
        if expected_value != observed_value:
            reasons.append(f"{name}_mismatch")
    if observed.frame_id != tolerances.expected_frame_id:
        reasons.append("frame_id_mismatch")
    if observed.stamp_ns <= 0:
        reasons.append("stamp_invalid")
    return ComparisonResult(matches=not reasons, reasons=tuple(reasons))
