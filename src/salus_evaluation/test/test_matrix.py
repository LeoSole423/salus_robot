import json
from pathlib import Path

import pytest

from salus_evaluation.matrix import (aggregate_trials, continuous_summary,
                                     expand_matrix, load_matrix,
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
