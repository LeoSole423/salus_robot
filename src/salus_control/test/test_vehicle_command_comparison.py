import math

import pytest

from salus_control.legacy_vehicle_command import VehicleCommandValue
from salus_control.vehicle_command_comparison import (
    ComparisonTolerances,
    ObservedVehicleCommand,
    compare_vehicle_commands,
)


def expected_command() -> VehicleCommandValue:
    return VehicleCommandValue(
        source=1,
        drive_enabled=True,
        emergency_stop=False,
        brake_ratio=0.0,
        speed_mps=2.0,
        steering_angle_rad=0.2,
        valid_for_s=0.7,
    )


def observed_command(**overrides) -> ObservedVehicleCommand:
    values = {
        "source": 1,
        "drive_enabled": True,
        "emergency_stop": False,
        "brake_ratio": 0.0,
        "speed_mps": 2.0,
        "steering_angle_rad": 0.2,
        "valid_for_s": 0.7,
        "frame_id": "base_footprint",
        "stamp_ns": 1,
    }
    values.update(overrides)
    return ObservedVehicleCommand(**values)


def test_equal_commands_match() -> None:
    result = compare_vehicle_commands(
        expected_command(), observed_command(), ComparisonTolerances()
    )

    assert result.matches
    assert result.reasons == ()


def test_scalar_tolerances_are_explicit() -> None:
    within = observed_command(speed_mps=2.000009)
    outside = observed_command(speed_mps=2.000011)

    assert compare_vehicle_commands(
        expected_command(), within, ComparisonTolerances()
    ).matches
    result = compare_vehicle_commands(
        expected_command(), outside, ComparisonTolerances()
    )
    assert not result.matches
    assert result.reasons == ("speed_mps_mismatch",)


def test_semantic_and_metadata_divergences_are_reported_together() -> None:
    observed = observed_command(
        source=2,
        emergency_stop=True,
        frame_id="wrong_frame",
        stamp_ns=0,
        brake_ratio=math.nan,
    )

    result = compare_vehicle_commands(
        expected_command(), observed, ComparisonTolerances()
    )

    assert not result.matches
    assert result.reasons == (
        "brake_ratio_nonfinite",
        "source_mismatch",
        "emergency_stop_mismatch",
        "frame_id_mismatch",
        "stamp_invalid",
    )


@pytest.mark.parametrize(
    "kwargs", [{"speed_mps": -1.0}, {"brake_ratio": math.inf}]
)
def test_invalid_tolerances_are_rejected(kwargs) -> None:
    with pytest.raises(ValueError):
        ComparisonTolerances(**kwargs)
