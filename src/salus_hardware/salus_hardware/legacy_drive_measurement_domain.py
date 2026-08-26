"""Pure translation of legacy drive telemetry to physical measurements.

The legacy UART telemetry reports a magnitude derived from the traction
motor/transmission and an AS5600 linkage position.  This module preserves
that boundary: it does not claim wheel-ground odometry or a road-wheel angle.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


STATUS_OK = 1
STATUS_STALE = 2
STATUS_INVALID = 3

TRACTION_SOURCE_MOTOR_SHAFT = 1
STEERING_SOURCE_LINKAGE = 2

FIELD_POSITION = 1
FIELD_ANGULAR_VELOCITY = 2
FIELD_LINEAR_VELOCITY = 4


@dataclass(frozen=True)
class LegacyDriveSample:
    """Values extracted from one ``DriveTelemetry`` message."""

    fresh: bool
    reverse_requested: bool
    speed_valid: bool
    steer_valid: bool
    speed_mps_measured: float
    steer_deg_measured: float


@dataclass(frozen=True)
class MeasurementFields:
    """Status, field availability, and field-level provenance masks."""

    status: int
    available_fields: int
    measured_fields: int
    calculated_fields: int
    inferred_fields: int


@dataclass(frozen=True)
class TractionMeasurementValue:
    source_type: int
    fields: MeasurementFields
    linear_velocity_mps: float


@dataclass(frozen=True)
class SteeringMeasurementValue:
    source_type: int
    fields: MeasurementFields
    position_rad: float


def adapt_legacy_drive_sample(
    sample: LegacyDriveSample,
) -> tuple[TractionMeasurementValue, SteeringMeasurementValue]:
    """Translate one legacy sample without assigning false sensor semantics.

    ``reverse_requested`` is an intent rather than an observation.  Both signs
    of the canonical velocity are therefore inferred from command state; an
    unasserted reverse request is not evidence of forward motion.  The linkage
    degree-to-radian conversion is calculated.
    """
    speed_is_valid = sample.speed_valid and math.isfinite(sample.speed_mps_measured)
    steer_is_valid = sample.steer_valid and math.isfinite(sample.steer_deg_measured)

    traction_fields = _fields(
        fresh=sample.fresh,
        valid=speed_is_valid,
        field=FIELD_LINEAR_VELOCITY,
        inferred=True,
    )
    steering_fields = _fields(
        fresh=sample.fresh,
        valid=steer_is_valid,
        field=FIELD_POSITION,
        inferred=False,
    )

    speed = math.nan
    if speed_is_valid:
        speed = (
            -abs(sample.speed_mps_measured)
            if sample.reverse_requested
            else abs(sample.speed_mps_measured)
        )

    steering = math.nan
    if steer_is_valid:
        steering = math.radians(sample.steer_deg_measured)

    return (
        TractionMeasurementValue(
            TRACTION_SOURCE_MOTOR_SHAFT,
            traction_fields,
            speed,
        ),
        SteeringMeasurementValue(
            STEERING_SOURCE_LINKAGE,
            steering_fields,
            steering,
        ),
    )


def _fields(*, fresh: bool, valid: bool, field: int, inferred: bool) -> MeasurementFields:
    if not valid:
        return MeasurementFields(STATUS_INVALID, 0, 0, 0, 0)
    status = STATUS_OK if fresh else STATUS_STALE
    return MeasurementFields(
        status,
        field,
        0,
        0 if inferred else field,
        field if inferred else 0,
    )
