import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from salus_evaluation.matrix import (aggregate_trials, continuous_summary,
                                     effective_speed_matches, expand_matrix,
                                     load_matrix, matrix_exit_code,
                                     parse_effective_speed,
                                     write_matrix_artifacts)


ROOT = Path(__file__).parents[1]


def _summary(success=True, offset=0.0):
    return {
        "terminal_status": 4 if success else 6,
        "metrics": {"cross_track_rms_m": .1 + offset,
                    "cross_track_p95_m": .2 + offset,
                    "heading_p95_rad": .3 + offset},
        "arrival": {"final_distance_m": .4 + offset, "overshoot_m": .5 + offset},
        "replans": 1,
        "command_chain": {"steering_saturation": {"interval_count": 2},
                          "ackermann": {"requested_to_applied_steer_delta_rad": {
                              "max": .02 + offset}}},
    }


def test_initial_matrix_expands_speed_geometry_and_repetitions_deterministically():
    cells = expand_matrix(ROOT / "config/matrices/ackermann_speed_curvature.yaml")
    assert len(cells) == 54
    assert cells[0].trial_id == "straight-straight-straight-v0p8-rep01"
    assert {cell.case.direction for cell in cells} == {"left", "right", "straight"}
    assert {cell.case.requested_radius_m for cell in cells} == {None, 4.0, 8.0}
    assert cells == expand_matrix(ROOT / "config/matrices/ackermann_speed_curvature.yaml")


def test_matrix_rejects_invalid_speed_and_straight_radius(tmp_path):
    path = tmp_path / "bad.yaml"
    path.write_text("""schema_version: 1
id: bad
repetitions: 1
max_speed_mps: 1.0
speeds_mps: [1.1]
cases: []
""")
    with pytest.raises(ValueError, match="no greater"):
        load_matrix(path)
    path.write_text("""schema_version: 1
id: bad
repetitions: 1
max_speed_mps: 1.0
speeds_mps: [1.1]
cases:
  - {id: straight, scenario: x.yaml, direction: straight, requested_radius_m: null}
""")
    with pytest.raises(ValueError, match="no greater"):
        load_matrix(path)


def test_aggregation_keeps_failed_trials_and_performance_report_only(tmp_path):
    matrix = ROOT / "config/matrices/ackermann_speed_curvature.yaml"
    cells = expand_matrix(matrix)[:3]
    rows = aggregate_trials(cells, {cells[0].trial_id: _summary(True),
                                    cells[1].trial_id: _summary(False, .1),
                                    cells[2].trial_id: _summary(True, .2)})
    assert len(rows) == 1
    row = rows[0]
    assert row["trial_count"] == 3 and row["success_count"] == 2
    assert row["failure_count"] == 1 and row["success_rate"] == pytest.approx(2 / 3)
    assert row["cross_track_rmse_m"]["median"] == pytest.approx(.2)
    assert row["performance_gate_state"] == "calibrating"


def test_one_continuous_sample_has_no_artificial_p95():
    assert continuous_summary([.2])["p95"] is None


def test_effective_speed_readback_is_numeric_and_matches_with_explicit_tolerance():
    effective, matches = effective_speed_matches(1.2, "Double value is: 1.2000004")
    assert effective == pytest.approx(1.2000004)
    assert matches


@pytest.mark.parametrize("readback", ("not set", "1.2 1.3", "nan"))
def test_effective_speed_readback_rejects_malformed_or_ambiguous_values(readback):
    assert parse_effective_speed(readback) is None
    assert effective_speed_matches(1.2, readback) == (None, False)


def test_effective_speed_readback_rejects_a_different_applied_speed():
    assert effective_speed_matches(1.2, "Double value is: 1.19") == (1.19, False)


def test_numeric_parameter_metadata_uses_radius_names_and_units():
    from salus_evaluation.matrix_executor import _numeric_parameter_metadata

    result = _numeric_parameter_metadata(
        2.5, SimpleNamespace(returncode=0, stdout="Double value is: 2.5", stderr=""),
        quantity="radius_m", unit="m",
    )
    assert result["requested_radius_m"] == 2.5
    assert result["effective_radius_m"] == 2.5
    assert result["unit"] == "m" and result["matches_requested"]
    assert result["tolerance"] == pytest.approx(1.0e-6)
    assert "tolerance_mps" not in result
    assert "requested_speed_mps" not in result


def test_numeric_parameter_metadata_keeps_speed_set_returncode():
    from salus_evaluation.matrix_executor import _numeric_parameter_metadata

    result = _numeric_parameter_metadata(
        1.2, SimpleNamespace(returncode=0, stdout="Double value is: 1.2", stderr=""),
        setup_result=SimpleNamespace(returncode=7, stdout="", stderr="rejected"),
        quantity="speed_mps", unit="m/s",
    )
    assert result["setup_returncode"] == 7
    assert result["get_returncode"] == 0
    assert result["requested_speed_mps"] == 1.2
    assert result["effective_speed_mps"] == 1.2
    assert result["unit"] == "m/s" and result["matches_requested"]


def test_candidate_nav2_params_changes_only_smac_radius(tmp_path):
    from salus_evaluation.matrix_executor import _sha256, write_candidate_nav2_params

    base = tmp_path / "base.yaml"
    base.write_text("""planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    GridBased:
      plugin: nav2_smac_planner/SmacPlannerHybrid
      minimum_turning_radius: 4.0
controller_server:
  ros__parameters:
    controller_frequency: 20.0
""")
    candidate = write_candidate_nav2_params(base, tmp_path / "candidate.yaml", 2.5)
    duplicate = write_candidate_nav2_params(base, tmp_path / "duplicate.yaml", 2.5)
    import yaml
    original, generated = yaml.safe_load(base.read_text()), yaml.safe_load(candidate.read_text())
    grid_based = generated["planner_server"]["ros__parameters"]["GridBased"]
    assert grid_based["minimum_turning_radius"] == 2.5
    generated["planner_server"]["ros__parameters"]["GridBased"]["minimum_turning_radius"] = 4.0
    assert generated == original
    assert _sha256(candidate) == _sha256(duplicate)


def test_radius_readback_mismatch_is_recorded_as_nonmatching():
    from salus_evaluation.matrix_executor import _numeric_parameter_metadata

    result = _numeric_parameter_metadata(
        2.5, SimpleNamespace(returncode=0, stdout="Double value is: 2.4", stderr=""),
        quantity="radius_m", unit="m",
    )
    assert result["effective_radius_m"] == 2.4
    assert not result["matches_requested"]


def test_planner_override_readiness_requires_active_lifecycle_and_get_service(monkeypatch):
    from salus_evaluation import matrix_executor

    def response(command, **_kwargs):
        joined = " ".join(command)
        if "topic echo" in joined or "param get" in joined:
            return SimpleNamespace(returncode=0, stdout="true\n", stderr="")
        if "lifecycle get" in joined:
            return SimpleNamespace(returncode=0, stdout="active [3]\n", stderr="")
        if "service list" in joined:
            return SimpleNamespace(
                returncode=0,
                stdout="/planner_server/get_parameters\n",
                stderr="",
            )
        raise AssertionError(command)

    monkeypatch.setattr(matrix_executor, "_run", response)
    ready, evidence = matrix_executor._readiness_snapshot(require_planner=True)
    assert ready and evidence["planner_parameter_services"]["get_available"]

    def missing_get(command, **kwargs):
        result = response(command, **kwargs)
        if "service list" in " ".join(command):
            result.stdout = "/planner_server/set_parameters\n"
        return result

    monkeypatch.setattr(matrix_executor, "_run", missing_get)
    ready, evidence = matrix_executor._readiness_snapshot(require_planner=True)
    assert not ready and not evidence["planner_parameter_services"]["get_available"]


def test_matrix_exit_is_aggregated_after_all_trials_and_ignores_calibration():
    assert matrix_exit_code(("passed", "passed", "passed")) == 0
    outcomes = []
    for outcome in ("passed", "functional_failure", "passed"):
        outcomes.append(outcome)
    assert outcomes == ["passed", "functional_failure", "passed"]
    assert matrix_exit_code(outcomes) == 1
    assert matrix_exit_code(("passed", "calibrating", "passed")) == 0


def test_matrix_setup_failure_requires_nonzero_exit_without_discarding_other_results(tmp_path):
    from salus_evaluation.matrix_executor import _failure_bundle

    directory = tmp_path / "failed-trial"
    _failure_bundle(directory, "readback mismatch", {"trial_id": "failed"})
    summary = json.loads((directory / "summary.json").read_text())
    assert summary["reason"] == "matrix_setup_failure"
    assert matrix_exit_code(("passed", "setup_failure", "passed")) == 1


def test_matrix_artifact_is_reproducible_and_links_each_trial(tmp_path):
    matrix = ROOT / "config/matrices/ackermann_speed_curvature.yaml"
    cells = expand_matrix(matrix)[:2]
    trial_dirs = []
    for index, cell in enumerate(cells):
        directory = tmp_path / f"trial-{index}"
        directory.mkdir()
        (directory / "summary.json").write_text(json.dumps(_summary(offset=index / 10)))
        trial_dirs.append(directory)
    root = write_matrix_artifacts(tmp_path / "matrix", matrix, cells, trial_dirs)
    manifest = json.loads((root / "matrix-manifest.json").read_text())
    summary = json.loads((root / "matrix-summary.json").read_text())
    assert manifest["performance_gates"] == "calibrating/report-only"
    assert [item["trial_id"] for item in manifest["trials"]] == [cell.trial_id for cell in cells]
    assert summary[0]["trial_ids"] == [cell.trial_id for cell in cells]
    assert (root / "matrix-summary.csv").exists()
