import json

from salus_evaluation.artifacts import write_artifacts
from salus_evaluation.gates import GateResult, GateState
from salus_evaluation.models import LocalizationCovarianceSummary


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


def test_artifacts_persist_localization_yaw_p95_and_covariance_summary(tmp_path):
    root = write_artifacts(
        tmp_path / "trial",
        {"schema_version": 2, "streams": ["odometry_local"]},
        {
            "schema_version": 2,
            "localization": {
                "yaw_p95_rad": .12,
            },
            "localization_covariance": LocalizationCovarianceSummary(
                4, .1, .2, .3, .4, .01, .02
            ),
        },
        {"odometry_local": []},
    )
    summary = json.loads((root / "summary.json").read_text())
    assert summary["localization"]["yaw_p95_rad"] == .12
    assert summary["localization_covariance"] == {
        "sample_count": 4,
        "x_m2_median": .1, "x_m2_p95": .2,
        "y_m2_median": .3, "y_m2_p95": .4,
        "yaw_rad2_median": .01, "yaw_rad2_p95": .02,
    }
