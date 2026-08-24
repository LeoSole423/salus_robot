from pathlib import Path


SCRIPT = Path(__file__).parents[3] / "tools" / "sim.sh"
OPERATIONAL_SCRIPT = Path(__file__).parents[3] / "tools" / "sim_operational.sh"


def test_cockpit_mode_delegates_to_the_operational_profile() -> None:
    contents = SCRIPT.read_text(encoding="utf-8")
    assert "--cockpit)" in contents
    assert "sim_operational.sh" in contents
    assert "Usage: ./tools/sim.sh [--headless] [--cockpit]" in contents


def test_operational_helper_exposes_the_canonical_profile() -> None:
    contents = OPERATIONAL_SCRIPT.read_text(encoding="utf-8")
    assert "sim_operational.launch.py" in contents
    assert "Usage: ./tools/sim_operational.sh [--headless] [--rviz]" in contents
    assert "ws://localhost:8766" in contents
