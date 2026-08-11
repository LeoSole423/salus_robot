from geometry_msgs.msg import Twist
from nav2_msgs.msg import CollisionMonitorState
from salus_interfaces.msg import CmdVelFinal

from salus_navigation.nav_command_server import CommandArbiter


def test_automatic_command_requires_fresh_safety_scan() -> None:
    arbiter = CommandArbiter(manual_timeout_s=0.4, monitor_timeout_s=1.0)
    twist = Twist()
    twist.linear.x = 1.0
    command, reason = arbiter.automatic_output(twist, 10.0)
    assert reason == "scan_stale"
    assert command.brake_pct == 0
    assert command.source == CmdVelFinal.SOURCE_SAFETY


def test_automatic_command_passes_with_a_fresh_safety_scan() -> None:
    arbiter = CommandArbiter(manual_timeout_s=0.4, monitor_timeout_s=1.0)
    arbiter.set_scan_received(5.0)
    twist = Twist()
    twist.linear.x = 0.75
    command, reason = arbiter.automatic_output(twist, 5.2)
    assert reason == "auto"
    assert command.twist.linear.x == 0.75
    assert command.source == CmdVelFinal.SOURCE_AUTO


def test_stop_state_overrides_automatic_command() -> None:
    arbiter = CommandArbiter(manual_timeout_s=0.4, monitor_timeout_s=1.0)
    state = CollisionMonitorState()
    state.action_type = CollisionMonitorState.STOP
    arbiter.set_scan_received(5.0)
    arbiter.set_monitor_state(state, 5.0)
    command, reason = arbiter.automatic_output(Twist(), 5.1)
    assert reason == "collision_stop"
    assert command.twist.linear.x == 0.0
    assert command.source == CmdVelFinal.SOURCE_SAFETY


def test_manual_mode_blocks_auto_and_times_out_once() -> None:
    arbiter = CommandArbiter(manual_timeout_s=0.4, monitor_timeout_s=1.0)
    arbiter.set_manual_mode(True)
    manual = CmdVelFinal()
    manual.twist.linear.x = 0.5
    manual.brake_pct = 200
    accepted = arbiter.accept_manual(manual, 1.0)
    assert accepted.brake_pct == 100
    assert accepted.source == CmdVelFinal.SOURCE_MANUAL
    assert arbiter.automatic_output(Twist(), 1.1) == (None, "manual_enabled")
    stop = arbiter.manual_watchdog_output(1.5)
    assert stop is not None
    assert stop.source == CmdVelFinal.SOURCE_MANUAL
    assert arbiter.manual_watchdog_output(1.6) is None
