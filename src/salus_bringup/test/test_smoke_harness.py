from pathlib import Path
import json
import subprocess


HARNESS = Path(__file__).parents[3] / "tools" / "smoke_harness.sh"


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
    workflow = (HARNESS.parents[1] / ".github/workflows/ci.yml").read_text(encoding="utf-8")
    assert 'SMOKE_HARD_TIMEOUT_S: "120"' in workflow
    assert 'SMOKE_HARD_TIMEOUT_S: "180"' in workflow
    heavy_smokes = (
        "smoke_navigation_core_sim.sh",
        "smoke_navigation_zones_sim.sh",
        "smoke_route_executor_sim.sh",
        "smoke_patrol_battery_sim.sh",
        "smoke_navigation_snapshot.sh",
    )
    for smoke in heavy_smokes:
        step_end = workflow.index(f"./tools/{smoke}")
        step_start = workflow.rfind("      - name:", 0, step_end)
        step = workflow[step_start:step_end]
        assert 'SMOKE_HARD_TIMEOUT_S: "240"' in step, smoke
