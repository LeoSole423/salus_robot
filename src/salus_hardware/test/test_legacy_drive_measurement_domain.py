import math

import pytest

from salus_hardware.legacy_drive_measurement_domain import (
    FIELD_LINEAR_VELOCITY,
    FIELD_POSITION,
    STATUS_INVALID,
    STATUS_OK,
    STATUS_STALE,
    LegacyDriveSample,
    adapt_legacy_drive_sample,
)


def sample(**changes: object) -> LegacyDriveSample:
    values = {
        "fresh": True,
        "reverse_requested": False,
        "speed_valid": True,
        "steer_valid": True,
        "speed_mps_measured": 1.25,
        "steer_deg_measured": 90.0,
    }
    values.update(changes)
    return LegacyDriveSample(**values)


def test_valid_forward_sample_has_inferred_sign_and_calculated_linkage() -> None:
    traction, steering = adapt_legacy_drive_sample(sample())

    assert traction.source_type == 1
    assert traction.linear_velocity_mps == 1.25
    assert traction.fields.status == STATUS_OK
    assert traction.fields.available_fields == FIELD_LINEAR_VELOCITY
    assert traction.fields.calculated_fields == 0
    assert traction.fields.inferred_fields == FIELD_LINEAR_VELOCITY
    assert steering.source_type == 2
    assert steering.position_rad == pytest.approx(math.pi / 2.0)
    assert steering.fields.status == STATUS_OK
    assert steering.fields.available_fields == FIELD_POSITION
    assert steering.fields.calculated_fields == FIELD_POSITION


def test_reverse_command_marks_derived_sign_as_inferred() -> None:
    traction, _ = adapt_legacy_drive_sample(
        sample(reverse_requested=True, speed_mps_measured=1.25)
    )

    assert traction.linear_velocity_mps == -1.25
    assert traction.fields.available_fields == FIELD_LINEAR_VELOCITY
    assert traction.fields.calculated_fields == 0
    assert traction.fields.inferred_fields == FIELD_LINEAR_VELOCITY


def test_forward_command_uses_absolute_legacy_magnitude_but_remains_inferred() -> None:
    traction, _ = adapt_legacy_drive_sample(sample(speed_mps_measured=-1.25))

    assert traction.linear_velocity_mps == 1.25
    assert traction.fields.calculated_fields == 0
    assert traction.fields.inferred_fields == FIELD_LINEAR_VELOCITY


def test_stale_valid_values_keep_their_fields_but_are_not_ok() -> None:
    traction, steering = adapt_legacy_drive_sample(sample(fresh=False))

    assert traction.fields.status == STATUS_STALE
    assert traction.fields.available_fields == FIELD_LINEAR_VELOCITY
    assert steering.fields.status == STATUS_STALE
    assert steering.fields.available_fields == FIELD_POSITION


@pytest.mark.parametrize(
    ("changes", "traction_invalid", "steering_invalid"),
    [
        ({"speed_valid": False}, True, False),
        ({"steer_valid": False}, False, True),
        ({"speed_mps_measured": math.nan}, True, False),
        ({"steer_deg_measured": math.inf}, False, True),
        ({"fresh": False, "speed_valid": False}, True, False),
    ],
)
def test_invalid_flags_and_nonfinite_values_remove_only_the_bad_field(
    changes: dict[str, object],
    traction_invalid: bool,
    steering_invalid: bool,
) -> None:
    traction, steering = adapt_legacy_drive_sample(sample(**changes))

    if traction_invalid:
        assert traction.fields.status == STATUS_INVALID
        assert traction.fields.available_fields == 0
        assert traction.fields.calculated_fields == 0
        assert traction.fields.inferred_fields == 0
        assert math.isnan(traction.linear_velocity_mps)
    else:
        assert traction.fields.status in {STATUS_OK, STATUS_STALE}
        assert traction.fields.available_fields == FIELD_LINEAR_VELOCITY
    if steering_invalid:
        assert steering.fields.status == STATUS_INVALID
        assert steering.fields.available_fields == 0
        assert steering.fields.calculated_fields == 0
        assert math.isnan(steering.position_rad)
    else:
        assert steering.fields.status in {STATUS_OK, STATUS_STALE}
        assert steering.fields.available_fields == FIELD_POSITION


def test_provenance_masks_are_disjoint_and_exhaust_available_fields() -> None:
    for legacy in (sample(), sample(reverse_requested=True), sample(fresh=False)):
        traction, steering = adapt_legacy_drive_sample(legacy)
        for fields in (traction.fields, steering.fields):
            assert fields.measured_fields == 0
            assert not (fields.calculated_fields & fields.inferred_fields)
            assert (
                fields.measured_fields
                | fields.calculated_fields
                | fields.inferred_fields
            ) == fields.available_fields
