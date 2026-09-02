"""Structural safety tests for the operator-facing localization shadow wrapper."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

import ast


LAUNCH = Path(__file__).parents[1] / "launch" / "real_localization_shadow.launch.py"
OBSERVATION = Path(__file__).parents[1] / "launch" / "real_observation.launch.py"


def _code_text(path: Path) -> str:
    """Source without module docstring or comments (prose documents exclusions)."""
    source = path.read_text(encoding="utf-8")
    lines = source.splitlines()
    tree = ast.parse(source)
    if tree.body:
        first = tree.body[0]
        if (
            isinstance(first, ast.Expr)
            and isinstance(first.value, ast.Constant)
            and isinstance(first.value.value, str)
        ):
            for index in range(first.lineno - 1, first.end_lineno):
                lines[index] = ""
    return "\n".join(
        "" if line.lstrip().startswith("#") else line for line in lines
    )


def _launch_module():
    spec = spec_from_file_location("real_localization_shadow", LAUNCH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_wrapper_composes_the_two_validated_profiles_and_nothing_else() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    assert '_include("salus_bringup", "real_observation.launch.py")' in contents
    assert (
        '_include("salus_localization", "localization_real_shadow.launch.py")' in contents
    )
    assert contents.count("_include(") == 3  # two includes + the helper definition
    assert "Node(" not in contents


def test_wrapper_declares_no_reconfigurable_authority() -> None:
    contents = LAUNCH.read_text(encoding="utf-8")
    for forbidden in (
        "DeclareLaunchArgument",
        "delivery_enabled",
        "delivery_backend",
        "publish_tf",
        "use_control",
        "launch_navigation",
        "command_input_mode",
    ):
        assert forbidden not in contents


def test_wrapper_excludes_actuation_hardware_and_global_authority() -> None:
    code = _code_text(LAUNCH).lower()
    for forbidden in (
        "controller_server",
        "serial",
        "uart",
        "mavros",
        "ntrip",
        "rslidar",
        "robosense",
        "nav2",
        "collision_monitor",
        "navsat",
        "robot_state_publisher",
        "/tf",
        "/cmd_vel_final",
        "ekf_node",
    ):
        assert forbidden not in code


def test_wrapper_preserves_the_observation_profile_unchanged() -> None:
    """The wrapper must not weaken the already-validated read-only profile."""
    observation = OBSERVATION.read_text(encoding="utf-8")
    assert '"delivery_backend": "disabled"' in observation
    assert '"delivery_enabled": "false"' in observation
    assert observation.count('"input_wire_type": "interfaces"') == 3
    assert 'default_value="/cmd_vel_final"' in observation
    assert '"output_topic": "/vehicle/command_shadow"' in observation


def test_wrapper_is_constructible_without_hardware() -> None:
    description = _launch_module().generate_launch_description()
    assert len(description.entities) == 2
