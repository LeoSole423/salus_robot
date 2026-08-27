import pytest

from salus_control.controller_server_node import validate_command_input_mode


def test_legacy_input_remains_valid_for_uart_and_simulation() -> None:
    assert validate_command_input_mode("legacy_cmd_vel", "uart") == "legacy_cmd_vel"
    assert (
        validate_command_input_mode("legacy_cmd_vel", "sim_gazebo")
        == "legacy_cmd_vel"
    )


def test_canonical_input_is_restricted_to_simulation() -> None:
    assert (
        validate_command_input_mode("canonical_vehicle_command", "sim_gazebo")
        == "canonical_vehicle_command"
    )
    with pytest.raises(ValueError, match="restricted to the sim_gazebo"):
        validate_command_input_mode("canonical_vehicle_command", "uart")


def test_unknown_input_mode_is_rejected_without_fallback() -> None:
    with pytest.raises(ValueError, match="command_input_mode"):
        validate_command_input_mode("automatic", "sim_gazebo")
