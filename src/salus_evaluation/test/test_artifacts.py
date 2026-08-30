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


def test_artifacts_preserve_the_legacy_commands_stream_and_new_stages(tmp_path):
    root = write_artifacts(
        tmp_path / "trial",
        {"schema_version": 2, "topics": ["/cmd_vel_safe"],
         "streams": ["commands", "commands_safe", "commands_final"]},
        {"schema_version": 2, "command_chain": {"first_divergent_stage": None}},
        {
            "commands": [{"stamp_s": 1.0, "stage": "cmd_vel"}],
            "commands_safe": [{"stamp_s": 1.1, "stage": "cmd_vel_safe"}],
            "commands_final": [], "vehicle_commands": [], "drive_telemetry": [],
            "controller_status": [], "controller_telemetry": [],
        },
    )
    assert (root / "commands.csv").exists()
    assert (root / "commands_safe.csv").exists()
    manifest = json.loads((root / "manifest.json").read_text())
    assert manifest["schema_version"] == 2
    assert manifest["streams"] == ["commands", "commands_safe", "commands_final"]
