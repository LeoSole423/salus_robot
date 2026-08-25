from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_observer_launch_has_no_command_or_tf_authority():
    contents = (ROOT / "launch" / "evaluation_observer.launch.py").read_text()
    assert "navigation_evaluation" in contents
    assert "cmd_vel" not in contents
    assert "tf" not in contents
    assert '"goal_tolerance_m", default_value="1.2"' in contents
    assert '"precision_target_m", default_value="0.25"' in contents


def test_tool_exposes_run_and_rviz_observe_modes():
    contents = (ROOT.parents[1] / "tools" / "nav_eval.sh").read_text()
    assert "run <scenario.yaml>" in contents
    assert "observe" in contents
    assert "ros2 run salus_evaluation navigation_evaluation" in contents
