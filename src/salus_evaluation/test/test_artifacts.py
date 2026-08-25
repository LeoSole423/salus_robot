import json

from salus_evaluation.artifacts import write_artifacts
from salus_evaluation.gates import GateResult, GateState


def test_artifacts_are_versioned_json_csv_and_html(tmp_path):
    root = write_artifacts(
        tmp_path / "trial",
        {"schema_version": 1, "mode": "run"},
        {"schema_version": 1,
         "gate": GateResult("x", GateState.PASS, "ok"),
         "missing": float("inf")},
        {"commands": [{"stamp_s": 1.0, "linear_x_mps": .2}]},
    )
    assert json.loads((root / "manifest.json").read_text())["schema_version"] == 1
    assert json.loads((root / "summary.json").read_text())["missing"] is None
    assert "linear_x_mps" in (root / "commands.csv").read_text()
    assert "SALUS navigation evaluation" in (root / "report.html").read_text()
