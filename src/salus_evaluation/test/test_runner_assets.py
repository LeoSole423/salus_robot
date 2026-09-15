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


def test_fillet_arm_is_explicitly_evaluation_only():
    contents = (ROOT / "salus_evaluation" / "evaluation_runner.py").read_text()
    assert '"geometry_variant", "hard_vertex_current"' in contents
    assert '"hard_vertex_current", "sparse_fillet_r4", "sparse_single_3"' in contents
    assert "valid_fillet_r4(" in contents
    assert "NavigateThroughPoses" in contents
    assert "geometry_reference" in contents
    assert "dispatched_poses" in contents


def test_boundary_variants_are_evaluation_only_and_keep_request_provenance():
    contents = (ROOT / "salus_evaluation" / "evaluation_runner.py").read_text()
    for variant in (
            "sparse_single_3", "sparse_single_4", "sparse_boundary_exit",
            "sparse_boundary_midarc", "track3_current_boundary",
            "track3_sparse_exit"):
        assert f'"{variant}"' in contents
    assert "request_index" in contents
    assert "goal_generation" in contents
    assert "request_records" in contents
    assert "robot_pose_at_dispatch" in contents
    assert "logical_points" in contents
    assert "min_robot_distance_to_logical_p1_m" in contents
    assert "V=(8,0)" not in contents


def test_exact_productive_replay_is_an_evaluation_only_five_pose_request():
    contents = (ROOT / "salus_evaluation" / "evaluation_runner.py").read_text()
    assert '"track3_exact_productive_replay"' in contents
    replay = ROOT / "config" / "replays" / "issue244_t0_rep02_chunk_b.json"
    assert replay.is_file()


def test_matched_arms_use_three_pose_through_poses():
    contents = (ROOT / "salus_evaluation" / "evaluation_runner.py").read_text()
    assert "action_goal.poses.append(pose)" in contents
    assert "self.dispatched_poses = tuple" in contents
    assert "arc_points" in contents
    assert "action_goal.poses" in contents


def test_tool_exposes_run_observe_and_matrix_modes():
    contents = (ROOT.parents[1] / "tools" / "nav_eval.sh").read_text()
    assert "run <scenario.yaml>" in contents
    assert "observe" in contents
    assert "matrix <matrix.yaml>" in contents
    assert "navigation_matrix_execute" in contents
    assert "ros2 run salus_evaluation navigation_evaluation" in contents
    assert "isolation <output-dir>" in contents
    assert "SALUS_NAV_EVAL_RUN_TOKEN" in contents
    assert "SALUS_NAV_EVAL_LOCK_ROOT" in contents


def test_isolation_characterization_uses_shared_domain_allocator():
    contents = (ROOT.parents[1] / "tools" / "nav_eval_isolation.py").read_text()
    assert "allocated_trial_isolation" in contents
    assert "ExitStack" in contents
    assert "--domain-a" not in contents and "--domain-b" not in contents


def test_isolation_readiness_uses_typed_topic_probe():
    contents = (ROOT.parents[1] / "tools" / "nav_eval_isolation.py").read_text()
    assert "ros_topic_probe.py" in contents
    assert '"ros2", "topic", "echo"' not in contents
    assert '"typed_topic_probe"' in contents
