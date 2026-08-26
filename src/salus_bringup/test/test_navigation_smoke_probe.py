from pathlib import Path


ROOT = Path(__file__).parents[3]


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
