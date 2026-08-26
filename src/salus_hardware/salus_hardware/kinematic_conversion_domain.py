"""Pure calibrated conversion from physical observations to kinematic inputs."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


STATUS_OK = 1
STATUS_STALE = 2
STATUS_INVALID = 3
STATUS_UNAVAILABLE = 4

TRACTION_SOURCE_MOTOR_SHAFT = 1
TRACTION_SOURCE_DRIVE_WHEEL = 2
STEERING_SOURCE_LINKAGE = 2
STEERING_SOURCE_VIRTUAL_CENTER_WHEEL = 4

FIELD_POSITION = 1
FIELD_LINEAR_VELOCITY = 4


@dataclass(frozen=True)
class ConversionConfig:
    """Validated mechanical calibration for one converter instance."""

    calibration_validated: bool
    traction_linear_scale: float
    steering_coefficients: tuple[float, ...]
    steering_limit_rad: float

    @classmethod
    def create(
        cls,
        *,
        calibration_validated: bool,
        traction_linear_scale: float,
        steering_coefficients: Sequence[float],
        steering_limit_rad: float,
    ) -> "ConversionConfig":
        coefficients = tuple(float(value) for value in steering_coefficients)
        if not math.isfinite(traction_linear_scale) or traction_linear_scale <= 0.0:
            raise ValueError("traction_linear_scale must be finite and positive")
        if not 1 <= len(coefficients) <= 6 or not all(
            math.isfinite(value) for value in coefficients
        ):
            raise ValueError("steering_coefficients must contain 1..6 finite values")
        if not math.isfinite(steering_limit_rad) or steering_limit_rad <= 0.0:
            raise ValueError("steering_limit_rad must be finite and positive")
        return cls(
            bool(calibration_validated),
            float(traction_linear_scale),
            coefficients,
            float(steering_limit_rad),
        )


@dataclass(frozen=True)
class MeasurementInput:
    source_type: int
    status: int
    available_fields: int
    measured_fields: int
    calculated_fields: int
    inferred_fields: int
    value: float


@dataclass(frozen=True)
class ConvertedValue:
    source_type: int
    status: int
    available_fields: int
    measured_fields: int
    calculated_fields: int
    inferred_fields: int
    value: float


def convert_traction(
    sample: MeasurementInput, config: ConversionConfig
) -> ConvertedValue:
    """Convert motor/transmission linear magnitude to drive-wheel input."""
    return _convert(
        sample,
        config,
        expected_source=TRACTION_SOURCE_MOTOR_SHAFT,
        output_source=TRACTION_SOURCE_DRIVE_WHEEL,
        field=FIELD_LINEAR_VELOCITY,
        transform=lambda value: value * config.traction_linear_scale,
    )


def convert_steering(
    sample: MeasurementInput, config: ConversionConfig
) -> ConvertedValue:
    """Convert linkage angle to the virtual center-wheel Ackermann angle."""
    def transform(value: float) -> float:
        converted = 0.0
        for coefficient in reversed(config.steering_coefficients):
            converted = converted * value + coefficient
        return max(-config.steering_limit_rad, min(config.steering_limit_rad, converted))

    return _convert(
        sample,
        config,
        expected_source=STEERING_SOURCE_LINKAGE,
        output_source=STEERING_SOURCE_VIRTUAL_CENTER_WHEEL,
        field=FIELD_POSITION,
        transform=transform,
    )


def _convert(
    sample: MeasurementInput,
    config: ConversionConfig,
    *,
    expected_source: int,
    output_source: int,
    field: int,
    transform,
) -> ConvertedValue:
    if not config.calibration_validated:
        return _empty(output_source, STATUS_UNAVAILABLE)
    if sample.status != STATUS_OK:
        status = sample.status if sample.status in {
            STATUS_STALE, STATUS_INVALID, STATUS_UNAVAILABLE
        } else STATUS_INVALID
        return _empty(output_source, status)
    if sample.source_type != expected_source or not math.isfinite(sample.value):
        return _empty(output_source, STATUS_INVALID)
    if not _valid_provenance(sample, field):
        return _empty(output_source, STATUS_INVALID)
    output = float(transform(sample.value))
    if not math.isfinite(output):
        return _empty(output_source, STATUS_INVALID)
    inferred = field if sample.inferred_fields & field else 0
    calculated = 0 if inferred else field
    return ConvertedValue(
        output_source, STATUS_OK, field, 0, calculated, inferred, output
    )


def _valid_provenance(sample: MeasurementInput, field: int) -> bool:
    masks = (sample.measured_fields, sample.calculated_fields, sample.inferred_fields)
    if sample.available_fields != field:
        return False
    if any(mask & ~sample.available_fields for mask in masks):
        return False
    if (masks[0] & masks[1]) or (masks[0] & masks[2]) or (masks[1] & masks[2]):
        return False
    return (masks[0] | masks[1] | masks[2]) == sample.available_fields


def _empty(source_type: int, status: int) -> ConvertedValue:
    return ConvertedValue(source_type, status, 0, 0, 0, 0, math.nan)
