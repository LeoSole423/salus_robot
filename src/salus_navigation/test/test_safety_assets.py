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


def test_explicit_no_obstacle_launch_preserves_safe_command_boundary() -> None:
    launch = (
        PACKAGE / "launch" / "safety_arbitration_no_obstacles_sim.launch.py"
    ).read_text(encoding="utf-8")
    relay = (
        PACKAGE / "salus_navigation" / "safety_command_passthrough.py"
    ).read_text(encoding="utf-8")
    assert "nav2_collision_monitor" not in launch
    assert '"obstacle_detection_required": False' in launch
    assert '"/cmd_vel"' in relay
    assert '"/cmd_vel_safe"' in relay
    assert "collision" in relay


def test_no_obstacle_nav2_profile_disables_both_obstacle_layers() -> None:
    config = (
        PACKAGE / "config" / "nav2_core_no_obstacles_sim.yaml"
    ).read_text(encoding="utf-8")
    local = config.split("local_costmap:", 1)[1].split("global_costmap:", 1)[0]
    global_map = config.split("global_costmap:", 1)[1]
    assert "obstacle_layer:" in local and "enabled: false" in local
    assert "obstacle_layer:" in global_map and "enabled: false" in global_map
    assert config.count("vector_keepout_layer:") == 2
