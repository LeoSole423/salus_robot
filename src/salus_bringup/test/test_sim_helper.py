from pathlib import Path


SCRIPT = Path(__file__).parents[3] / "tools" / "sim.sh"


def test_cockpit_mode_enables_only_the_required_optional_subsystems() -> None:
    contents = SCRIPT.read_text(encoding="utf-8")
    assert "--cockpit)" in contents
    assert "launch_routes:=true" in contents
    assert "launch_patrol:=true" in contents
    assert "launch_web:=true" in contents
    assert "web_ws_port:=8766" in contents
    assert "Usage: ./tools/sim.sh [--headless] [--cockpit]" in contents
