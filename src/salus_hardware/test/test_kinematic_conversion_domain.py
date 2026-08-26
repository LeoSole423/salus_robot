import math

import pytest

from salus_hardware.kinematic_conversion_domain import (
    FIELD_LINEAR_VELOCITY,
    FIELD_POSITION,
    STATUS_INVALID,
    STATUS_OK,
    STATUS_STALE,
    STATUS_UNAVAILABLE,
    ConversionConfig,
    MeasurementInput,
    convert_steering,
    convert_traction,
)


def config(**changes) -> ConversionConfig:
    values = {
        "calibration_validated": True,
        "traction_linear_scale": 0.5,
        "steering_coefficients": (0.1, 2.0),
        "steering_limit_rad": 0.6,
    }
    values.update(changes)
    return ConversionConfig.create(**values)


def sample(*, field=FIELD_LINEAR_VELOCITY, source_type=1, provenance="calculated",
           status=STATUS_OK, value=2.0) -> MeasurementInput:
    masks = {"measured": (field, 0, 0), "calculated": (0, field, 0),
             "inferred": (0, 0, field)}[provenance]
    return MeasurementInput(source_type, status, field, *masks, value)


@pytest.mark.parametrize(
    "changes",
    [
        {"traction_linear_scale": 0.0},
        {"traction_linear_scale": math.inf},
        {"steering_coefficients": ()},
        {"steering_coefficients": (0.0,) * 7},
        {"steering_coefficients": (math.nan,)},
        {"steering_limit_rad": -1.0},
    ],
)
def test_calibration_validation_rejects_invalid_parameters(changes) -> None:
    with pytest.raises(ValueError):
        config(**changes)


def test_unvalidated_calibration_exposes_no_consumable_value() -> None:
    result = convert_traction(sample(), config(calibration_validated=False))
    assert result.status == STATUS_UNAVAILABLE
    assert result.available_fields == 0
    assert math.isnan(result.value)


def test_traction_scale_preserves_inferred_provenance() -> None:
    result = convert_traction(sample(provenance="inferred", value=-2.0), config())
    assert result.status == STATUS_OK
    assert result.source_type == 2
    assert result.value == -1.0
    assert result.inferred_fields == FIELD_LINEAR_VELOCITY
    assert result.calculated_fields == result.measured_fields == 0


@pytest.mark.parametrize("provenance", ["measured", "calculated"])
def test_conversion_outputs_calculated_for_noninferred_input(provenance) -> None:
    result = convert_traction(sample(provenance=provenance), config())
    assert result.calculated_fields == FIELD_LINEAR_VELOCITY
    assert result.measured_fields == result.inferred_fields == 0


def test_steering_polynomial_uses_radians_and_clamps() -> None:
    steering = sample(field=FIELD_POSITION, source_type=2, value=0.4)
    result = convert_steering(steering, config())
    assert result.source_type == 4
    assert result.value == pytest.approx(0.6)
    assert result.calculated_fields == FIELD_POSITION


@pytest.mark.parametrize("status", [STATUS_STALE, STATUS_INVALID, STATUS_UNAVAILABLE])
def test_non_ok_status_is_propagated_without_a_field(status) -> None:
    result = convert_traction(sample(status=status), config())
    assert result.status == status
    assert result.available_fields == 0
    assert math.isnan(result.value)


@pytest.mark.parametrize(
    "bad",
    [
        MeasurementInput(99, STATUS_OK, FIELD_LINEAR_VELOCITY, 0,
                         FIELD_LINEAR_VELOCITY, 0, 1.0),
        MeasurementInput(1, STATUS_OK, 0, 0, 0, 0, 1.0),
        MeasurementInput(1, STATUS_OK, FIELD_LINEAR_VELOCITY,
                         FIELD_LINEAR_VELOCITY, FIELD_LINEAR_VELOCITY, 0, 1.0),
        MeasurementInput(1, STATUS_OK, FIELD_LINEAR_VELOCITY, 0, 0, 0, 1.0),
        MeasurementInput(1, STATUS_OK, FIELD_LINEAR_VELOCITY, 0,
                         FIELD_LINEAR_VELOCITY, 0, math.nan),
    ],
)
def test_bad_type_field_provenance_or_value_is_invalid(bad) -> None:
    result = convert_traction(bad, config())
    assert result.status == STATUS_INVALID
    assert result.available_fields == 0
    assert math.isnan(result.value)
