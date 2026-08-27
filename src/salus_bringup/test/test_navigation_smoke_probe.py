import importlib.util
from pathlib import Path
import sys

from action_msgs.msg import GoalStatus, GoalStatusArray


ROOT = Path(__file__).parents[3]
PROBE = ROOT / "tools" / "smoke_navigation_core_sim.py"
sys.path.insert(0, str(PROBE.parent))
SPEC = importlib.util.spec_from_file_location("smoke_navigation_core_sim", PROBE)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


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


def test_navigation_action_idle_requires_a_terminal_status_snapshot() -> None:
    assert not probe.navigate_action_is_idle([])
    active = GoalStatusArray()
    active.status_list = [GoalStatus(status=GoalStatus.STATUS_CANCELING)]
    assert not probe.navigate_action_is_idle([active])
    terminal = GoalStatusArray()
    terminal.status_list = [GoalStatus(status=GoalStatus.STATUS_CANCELED)]
    assert probe.navigate_action_is_idle([active, terminal])


def test_second_goal_waits_for_nav2_cancel_completion() -> None:
    source = PROBE.read_text(encoding="utf-8")
    cancellation = source.split(
        '"right-turn diagnostic goal remained active"', 1
    )[1].split("start = node.odom[-1]", 1)[0]
    assert "navigate_action_is_idle" in cancellation
