import math

import pytest

from salus_control.control_logic import (
    COMMAND_SOURCE_AUTO,
    COMMAND_SOURCE_MANUAL,
    COMMAND_SOURCE_SAFETY,
)
from salus_control.legacy_vehicle_command import (
    LegacyVehicleCommandConfig,
    translate_legacy_command,
)


def test_forward_command_uses_physical_ackermann_units() -> None:
    command = translate_legacy_command(
        linear_x_mps=2.0,
        angular_z_rps=0.4,
        brake_pct=0,
        source=COMMAND_SOURCE_AUTO,
        config=LegacyVehicleCommandConfig(),
    )

    assert command.valid_input
    assert command.drive_enabled
    assert not command.emergency_stop
    assert command.brake_ratio == 0.0
    assert command.speed_mps == 2.0
    assert command.steering_angle_rad == pytest.approx(math.atan(0.94 * 0.2))


def test_reverse_speed_remains_signed() -> None:
    command = translate_legacy_command(
        linear_x_mps=-0.8,
        angular_z_rps=0.0,
        brake_pct=0,
        source=COMMAND_SOURCE_AUTO,
        config=LegacyVehicleCommandConfig(),
    )

    assert command.speed_mps == -0.8


def test_source_specific_steering_limit_is_preserved() -> None:
    config = LegacyVehicleCommandConfig()
    auto = translate_legacy_command(
        linear_x_mps=1.0,
        angular_z_rps=10.0,
        brake_pct=0,
        source=COMMAND_SOURCE_AUTO,
        config=config,
    )
    manual = translate_legacy_command(
        linear_x_mps=1.0,
        angular_z_rps=10.0,
        brake_pct=0,
        source=COMMAND_SOURCE_MANUAL,
        config=config,
    )

    assert auto.steering_angle_rad == pytest.approx(
        config.operational_steering_limit_rad
    )
    assert manual.steering_angle_rad == pytest.approx(
        config.manual_operational_steering_limit_rad
    )


@pytest.mark.parametrize("brake_pct, expected_ratio", [(30, 0.3), (250, 1.0)])
def test_legacy_brake_is_explicitly_mapped_to_estop(
    brake_pct: int, expected_ratio: float
) -> None:
    command = translate_legacy_command(
        linear_x_mps=2.0,
        angular_z_rps=0.5,
        brake_pct=brake_pct,
        source=COMMAND_SOURCE_AUTO,
        config=LegacyVehicleCommandConfig(),
    )

    assert command.emergency_stop
    assert command.brake_ratio == expected_ratio
    assert command.speed_mps == 0.0
    assert command.steering_angle_rad == 0.0


@pytest.mark.parametrize("linear_x, angular_z", [(math.nan, 0.0), (0.0, math.inf)])
def test_nonfinite_input_produces_safe_shadow_command(
    linear_x: float, angular_z: float
) -> None:
    command = translate_legacy_command(
        linear_x_mps=linear_x,
        angular_z_rps=angular_z,
        brake_pct=0,
        source=COMMAND_SOURCE_AUTO,
        config=LegacyVehicleCommandConfig(),
    )

    assert not command.valid_input
    assert command.source == COMMAND_SOURCE_SAFETY
    assert not command.drive_enabled
    assert command.emergency_stop
    assert command.brake_ratio == 1.0
    assert command.speed_mps == 0.0
    assert command.steering_angle_rad == 0.0


def test_unknown_enum_value_produces_safe_shadow_command() -> None:
    command = translate_legacy_command(
        linear_x_mps=1.0,
        angular_z_rps=0.0,
        brake_pct=0,
        source=99,
        config=LegacyVehicleCommandConfig(),
    )

    assert not command.valid_input
    assert command.source == COMMAND_SOURCE_SAFETY
    assert command.emergency_stop


@pytest.mark.parametrize(
    "kwargs",
    [
        {"valid_for_s": 0.0},
        {"wheelbase_m": -1.0},
        {"max_speed_mps": math.nan},
    ],
)
def test_invalid_configuration_is_rejected(kwargs: dict[str, float]) -> None:
    with pytest.raises(ValueError):
        LegacyVehicleCommandConfig(**kwargs)
