from pathlib import Path


PACKAGE = Path(__file__).parents[1]


def test_safety_launch_owns_collision_monitor_lifecycle_and_arbiter() -> None:
    launch = (PACKAGE / "launch" / "safety_arbitration_sim.launch.py").read_text(encoding="utf-8")
    assert "nav2_collision_monitor" in launch
    assert "nav2_lifecycle_manager" in launch
    assert "nav_command_server" in launch


def test_collision_monitor_uses_the_canonical_clean_scan() -> None:
    config = (PACKAGE / "config" / "collision_monitor.yaml").read_text(encoding="utf-8")
    assert "topic: /scan_clean" in config
    for polygon in ("footprint", "stop_zone", "critical_slow_zone", "slow_zone"):
        assert polygon in config
