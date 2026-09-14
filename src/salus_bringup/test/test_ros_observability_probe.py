"""Pure timestamp tests for the bounded read-only observability probe."""

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[3]
PROBE_PATH = ROOT / "tools" / "ros_observability_probe.py"
SPEC = importlib.util.spec_from_file_location("ros_observability_probe", PROBE_PATH)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def test_age_is_signed_and_unstamped_data_has_no_fake_age() -> None:
    assert probe.age_seconds(2_000_000_000, 1_250_000_000) == pytest.approx(0.75)
    assert probe.age_seconds(1_000_000_000, 1_250_000_000) == pytest.approx(-0.25)
    assert probe.age_seconds(0, 1_000_000_000) is None
    assert probe.age_seconds(1_000_000_000, None) is None


def test_timestamp_gaps_preserve_non_monotonicity() -> None:
    assert probe.timestamp_gap_seconds(1_000_000_000, 1_200_000_000) == pytest.approx(0.2)
    assert probe.timestamp_gap_seconds(1_200_000_000, 1_100_000_000) == pytest.approx(-0.1)
    assert probe.timestamp_gap_seconds(None, 1_100_000_000) is None


def test_interpolation_is_inside_window_and_rejects_extrapolation() -> None:
    samples = [(1_000_000_000, 10.0), (2_000_000_000, 20.0)]
    assert probe.interpolate_scalar(samples, 1_500_000_000) == pytest.approx(15.0)
    assert probe.interpolate_scalar(samples, 1_000_000_000) == pytest.approx(10.0)
    assert probe.interpolate_scalar(samples, 500_000_000) is None
    assert probe.interpolate_scalar(samples, 2_500_000_000) is None
    reversed_samples = [(2_000_000_000, 20.0), (1_000_000_000, 10.0)]
    assert probe.interpolate_scalar(reversed_samples, 1_500_000_000) is None


def test_summary_separates_receipt_and_source_gaps() -> None:
    rows = [
        {"topic": "/scan", "receipt_steady_ns": 100, "source_stamp_ns": 1_000, "age_s": 0.1},
        {"topic": "/scan", "receipt_steady_ns": 350, "source_stamp_ns": 1_200, "age_s": 0.2},
        {"topic": "/scan", "receipt_steady_ns": 500, "source_stamp_ns": 1_100, "age_s": 0.3},
        {
            "topic": "/cmd_vel_safe",
            "receipt_steady_ns": 700,
            "source_stamp_ns": None,
            "age_s": None,
        },
    ]
    summary = probe.summarize_rows(rows)
    assert summary["/scan"]["max_receipt_gap_s"] == pytest.approx(2.5e-7)
    assert summary["/scan"]["max_source_gap_s"] == pytest.approx(2e-7)
    assert summary["/scan"]["source_backwards_or_equal"] == 1
    assert summary["/cmd_vel_safe"]["latest_age_s"] is None


def test_cli_requires_a_bounded_output_and_positive_duration() -> None:
    with pytest.raises(SystemExit):
        probe.parse_args([])
    with pytest.raises(SystemExit):
        probe.parse_args(["--json-out", "/tmp/probe.json", "--duration-s", "0"])
    args = probe.parse_args(["--csv-out", "/tmp/probe.csv", "--duration-s", "1"])
    assert args.duration_s == 1.0


def test_probe_topics_are_read_only_and_cover_the_causal_window() -> None:
    source = PROBE_PATH.read_text(encoding="utf-8")
    assert "create_publisher" not in source
    assert "create_client" not in source
    for topic in (
        "/scan_3d",
        "/obstacles_cloud",
        "/scan",
        "/scan_clean",
        "/cmd_vel_safe",
        "/cmd_vel_final",
        "/gps/course_heading/debug",
        "/gps/course_heading",
        "/localization/orientation",
        "/odometry/global",
        "/nav_command_server/events",
        "/nav_command_server/telemetry",
        "map",
        "odom",
        "base_footprint",
    ):
        assert topic in source


def test_report_writer_preserves_provenance_in_json_and_csv(tmp_path: Path) -> None:
    report = {
        "schema_version": 1,
        "capture_mode": "synthetic_fixture",
        "provenance": {
            "tool": "ros_observability_probe",
            "schema_version": 1,
            "source_sha": "abc123",
            "source_branch": "agent/test",
            "capture_mode": "synthetic_fixture",
        },
        "rows": [
            {
                "topic": "/scan",
                "receipt_steady_ns": 1,
                "source_stamp_ns": 1,
                "age_s": 0.0,
            }
        ],
    }
    json_path = tmp_path / "capture.json"
    csv_path = tmp_path / "capture.csv"
    probe.write_report(report, str(json_path), str(csv_path))
    assert '"source_sha": "abc123"' in json_path.read_text(encoding="utf-8")
    csv_text = csv_path.read_text(encoding="utf-8")
    assert "# source_sha=abc123" in csv_text
    assert csv_text.splitlines()[5].startswith("sequence,topic,message_kind")
