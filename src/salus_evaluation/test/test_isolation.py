from pathlib import Path

from salus_evaluation.isolation import (
    EVALUATION_DOMAIN_MAX,
    EVALUATION_DOMAIN_MIN,
    TrialIsolation,
    allocated_trial_isolation,
    build_trial_env,
    make_trial_isolation,
)


def test_default_allocator_pool_is_below_linux_safe_upper_bound():
    assert 1 <= EVALUATION_DOMAIN_MIN <= EVALUATION_DOMAIN_MAX < 102


def test_trial_environment_sets_both_fortress_and_current_partition_names(tmp_path):
    isolation = make_trial_isolation(tmp_path, run_token="matrix", trial_id="trial-a",
                                     ros_domain_id=64)
    environment = build_trial_env({"ROS_DOMAIN_ID": "47", "KEEP": "yes"}, isolation)
    assert environment["ROS_DOMAIN_ID"] == "64"
    assert environment["IGN_PARTITION"] == environment["GZ_PARTITION"]
    assert environment["FASTDDS_BUILTIN_TRANSPORTS"] == "UDPv4"
    assert environment["ROS_LOG_DIR"] == str(isolation.ros_log_dir)
    assert environment["KEEP"] == "yes"


def test_trial_identities_have_distinct_domains_partitions_and_paths(tmp_path):
    first = make_trial_isolation(tmp_path, run_token="matrix", trial_id="trial-a",
                                 ros_domain_id=64)
    second = make_trial_isolation(tmp_path, run_token="matrix", trial_id="trial-b",
                                  ros_domain_id=65)
    assert first.ros_domain_id != second.ros_domain_id
    assert first.partition != second.partition
    assert first.runtime_root != second.runtime_root
    assert first.ros_log_dir != second.ros_log_dir


def test_trial_isolation_rejects_invalid_domain_and_shared_log_root(tmp_path):
    try:
        TrialIsolation(233, "partition", tmp_path / "runtime", tmp_path / "log")
    except ValueError as exc:
        assert "Linux-safe" in str(exc)
    else:
        raise AssertionError("invalid domain accepted")
    try:
        TrialIsolation(180, "partition", Path(tmp_path), Path(tmp_path))
    except ValueError as exc:
        assert "distinct" in str(exc)
    else:
        raise AssertionError("shared paths accepted")


def test_allocator_holds_domain_until_trial_finishes_and_reuses_after_release(tmp_path):
    with allocated_trial_isolation(
        tmp_path / "trial-a", run_token="matrix", trial_id="trial-a",
        lock_root=tmp_path, domain_min=120, domain_max=120,
    ) as first:
        assert first.ros_domain_id == 120
        try:
            with allocated_trial_isolation(
                tmp_path / "trial-b", run_token="matrix", trial_id="trial-b",
                lock_root=tmp_path, domain_min=120, domain_max=120,
            ):
                raise AssertionError("allocator reused a live domain")
        except RuntimeError as exc:
            assert "no evaluation ROS domain" in str(exc)
    with allocated_trial_isolation(
        tmp_path / "trial-c", run_token="matrix", trial_id="trial-c",
        lock_root=tmp_path, domain_min=120, domain_max=120,
    ) as released:
        assert released.ros_domain_id == 120
