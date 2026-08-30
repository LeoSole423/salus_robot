from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_observer_launch_has_no_command_or_tf_authority():
    contents = (ROOT / "launch" / "evaluation_observer.launch.py").read_text()
    assert "navigation_evaluation" in contents
    assert "cmd_vel" not in contents
    assert "tf" not in contents
    assert '"goal_tolerance_m", default_value="1.2"' in contents
    assert '"precision_target_m", default_value="0.25"' in contents


def test_runner_only_publishes_goal_and_markers_not_control_or_tf_topics():
    contents = (ROOT / "salus_evaluation" / "evaluation_runner.py").read_text()
    assert 'create_publisher(PoseStamped, "/goal_pose"' in contents
    assert 'create_publisher(MarkerArray, "/navigation_evaluation/markers"' in contents
    assert 'create_publisher(Twist, "/cmd_vel' not in contents
    assert 'create_publisher(VehicleCommand, "/vehicle/command' not in contents
    assert '"/tf"' not in contents and '"/tf_static"' not in contents


def test_tool_exposes_run_observe_and_matrix_modes():
    contents = (ROOT.parents[1] / "tools" / "nav_eval.sh").read_text()
    assert "run <scenario.yaml>" in contents
    assert "observe" in contents
    assert "matrix <matrix.yaml>" in contents
    assert "navigation_matrix_execute" in contents
    assert "ros2 run salus_evaluation navigation_evaluation" in contents
