from pathlib import Path
import json
import subprocess


HARNESS = Path(__file__).parents[3] / "tools" / "smoke_harness.sh"
RUNNER = HARNESS.parent / "run_smoke.sh"
COMPOSE = HARNESS.parents[1] / "compose.yaml"


def test_duplicate_artifact_name_is_rejected(tmp_path) -> None:
    command = f"""
      source {HARNESS}
      export SMOKE_ARTIFACT_ROOT={tmp_path}
      smoke_init duplicate-fixture
      smoke_reserve_artifact_name repeated
      smoke_reserve_artifact_name repeated
    """
    result = subprocess.run(["bash", "-c", command], text=True, capture_output=True)
    assert result.returncode != 0
    assert "already reserved" in result.stderr


def test_success_cleanup_is_lightweight_and_reports_timing(tmp_path) -> None:
    marker = tmp_path / "heavy-diagnostics-ran"
    command = f"""
      source {HARNESS}
      export SMOKE_ARTIFACT_ROOT={tmp_path}
      smoke_init success-fixture
      smoke_collect_diagnostics() {{ touch {marker}; }}
      smoke_cleanup
    """
    subprocess.run(["bash", "-c", command], check=True)
    assert not marker.exists()
    reports = list(tmp_path.glob("success-fixture-*/report.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text(encoding="utf-8"))
    assert report["status"] == "0"
    assert set(report["timing"]) == {
        "startup_s", "functional_s", "pre_cleanup_s", "cleanup_s", "total_s"
    }


def test_failure_cleanup_collects_full_diagnostics(tmp_path) -> None:
    marker = tmp_path / "heavy-diagnostics-ran"
    command = f"""
      source {HARNESS}
      export SMOKE_ARTIFACT_ROOT={tmp_path}
      smoke_init failure-fixture
      smoke_collect_diagnostics() {{ touch {marker}; }}
      false
      smoke_cleanup
    """
    result = subprocess.run(["bash", "-c", command])
    assert result.returncode != 0
    assert marker.exists()


def test_ci_assigns_scenario_specific_hard_timeouts() -> None:
    registry_path = HARNESS.parents[1] / "tools/smoke_scenarios.json"
    registry = json.loads(registry_path.read_text(encoding="utf-8"))
    scenarios = {scenario["id"]: scenario for scenario in registry["scenarios"]}

    assert scenarios["control"]["timeouts_s"]["ci"] == 120
    assert scenarios["integration"]["timeouts_s"]["ci"] == 180
    for scenario_id in (
        "navigation",
        "navigation_canonical",
        "navigation_no_obstacles",
        "zones",
        "routes",
        "patrol_battery",
        "snapshot",
        "web_cockpit",
    ):
        assert scenarios[scenario_id]["timeouts_s"]["ci"] == 240


def test_smoke_runner_defaults_fastdds_to_udp_only_without_changing_compose_default() -> None:
    runner = RUNNER.read_text(encoding="utf-8")
    compose = COMPOSE.read_text(encoding="utf-8")
    assert 'SMOKE_FASTDDS_BUILTIN_TRANSPORTS:-UDPv4' in runner
    assert 'FASTDDS_BUILTIN_TRANSPORTS: ${FASTDDS_BUILTIN_TRANSPORTS:-DEFAULT}' in compose


def test_harness_reports_effective_fastdds_transport(tmp_path) -> None:
    command = f"""
      source {HARNESS}
      export SMOKE_ARTIFACT_ROOT={tmp_path}
      export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
      smoke_init transport-fixture
      smoke_cleanup
    """
    subprocess.run(["bash", "-c", command], check=True)
    reports = list(tmp_path.glob("transport-fixture-*/report.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text(encoding="utf-8"))
    assert report["isolation"]["fastdds_builtin_transports"] == "UDPv4"
