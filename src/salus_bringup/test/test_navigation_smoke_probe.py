from pathlib import Path


ROOT = Path(__file__).parents[3]
PROBE = ROOT / "tools" / "smoke_navigation_core_sim.py"


def test_rviz_goal_is_published_once_after_discovery_and_uses_fresh_commands():
    source = (ROOT / "tools" / "smoke_navigation_core_sim.py").read_text(
        encoding="utf-8")
    section = source.split("start = node.odom[-1]", 1)[1].split(
        "wait_for(node, lambda: distance_from", 1)[0]
    assert "get_subscription_count() >= 1" in section
    assert section.count("node.rviz_goal.publish(rviz_goal)") == 1
    assert "stimulate=" not in section
    assert "node.raw_commands.clear()" in section
    assert "node.safe_commands.clear()" in section
    assert "node.final.clear()" in section


def test_cancel_service_is_the_terminal_boundary_before_the_next_goal() -> None:
    source = PROBE.read_text(encoding="utf-8")
    cancellation = source.split(
        '"right-turn command did not produce a negative physical yaw response"', 1
    )[1].split("start = node.odom[-1]", 1)[0]
    assert "timeout_s=15.0" in cancellation
    assert "cancel service returned before the right-turn goal became terminal" in cancellation
    assert "/navigate_to_pose/_action/status" not in source
    assert "navigate_action_is_idle" not in source


def test_manual_takeover_remains_immediate_but_waits_for_terminal_nav2_state() -> None:
    source = PROBE.read_text(encoding="utf-8")
    takeover = source.split(
        'SetManualMode.Request(enabled=True)', 1
    )[1].split('SetManualMode.Request(enabled=False)', 1)[0]
    assert '"manual takeover gained command authority but Nav2 cancellation did not reach a terminal state"' in takeover
    assert "15.0" in takeover


def test_canonical_variant_preserves_nav_authority_and_checks_fresh_input() -> None:
    source = PROBE.read_text(encoding="utf-8")
    wrapper = (ROOT / "tools" / "smoke_navigation_canonical_sim.sh").read_text(
        encoding="utf-8"
    )
    assert "command_input_mode:=canonical_vehicle_command" in wrapper
    assert "EXPECT_CANONICAL_COMMAND=1" in wrapper
    assert 'create_publisher(CmdVelFinal, "/cmd_vel_final"' not in source
    assert 'VehicleCommand, "/vehicle/command_shadow"' in source
    assert 'String, "/controller/status"' in source
    assert 'status.get("input_mode") == "canonical_vehicle_command"' in source
    assert 'status.get("fresh") is True' in source


def test_no_obstacle_variant_is_explicit_and_has_no_fake_scan() -> None:
    source = PROBE.read_text(encoding="utf-8")
    wrapper = (ROOT / "tools" / "smoke_navigation_no_obstacles_sim.sh").read_text(
        encoding="utf-8"
    )
    assert "capability_profile:=no_obstacle_detection" in wrapper
    assert "EXPECT_NO_OBSTACLE_DETECTION=1" in wrapper
    assert 'node.count_publishers("/scan_clean") == 0' in source
    assert 'node.count_publishers("/cmd_vel_safe") == 1' in source
    assert "STATE_DISABLED_BY_PROFILE" in source
