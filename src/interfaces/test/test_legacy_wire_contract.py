"""Regression coverage for the deliberately minimal legacy wire package."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
EXPECTED_MESSAGES = {
    "CmdVelFinal.msg": """uint8 SOURCE_UNKNOWN=0
uint8 SOURCE_AUTO=1
uint8 SOURCE_MANUAL=2
uint8 SOURCE_SAFETY=3
geometry_msgs/Twist twist
uint8 brake_pct
uint8 source
""",
    "DriveTelemetry.msg": """builtin_interfaces/Time stamp
bool ready
bool fresh
bool drive_enabled
bool estop
bool reverse_requested
bool speed_valid
bool steer_valid
string control_source
float64 speed_mps_measured
float64 steer_deg_measured
uint8 brake_applied_pct
""",
}


def test_package_contains_only_the_approved_legacy_messages() -> None:
    messages = {
        path.name: path.read_text(encoding="utf-8")
        for path in (PACKAGE_ROOT / "msg").glob("*.msg")
    }

    assert messages == EXPECTED_MESSAGES


def test_readme_records_the_fixed_legacy_source_and_transient_scope() -> None:
    readme = (PACKAGE_ROOT / "README.md").read_text(encoding="utf-8")

    assert "f35834989b041f51dd325c626d2338e2232d9e53" in readme
    assert "not a canonical API" in readme
    assert "Adding another legacy type requires explicit hardware" in readme
