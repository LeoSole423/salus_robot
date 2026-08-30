import importlib.util
import math
from pathlib import Path
import sys


ROOT = Path(__file__).parents[3]
PROBE_PATH = ROOT / "tools" / "runtime_timing_probe.py"
SPEC = importlib.util.spec_from_file_location("runtime_timing_probe", PROBE_PATH)
assert SPEC and SPEC.loader
probe = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def test_stream_stats_report_frequency_gaps_and_clock_age() -> None:
    stats = probe.StreamStats("local_ekf")
    stats.record(1_000_000_000, 10.0)
    stats.record(1_040_000_000, 10.05)

    summary = stats.summary(1_100_000_000)
    assert math.isclose(summary["effective_wall_hz"], 20.0)
    assert math.isclose(summary["effective_stamp_hz"], 25.0)
    assert math.isclose(summary["max_wall_gap_s"], 0.05)
    assert math.isclose(summary["max_stamp_gap_s"], 0.04)
    assert math.isclose(summary["age_vs_clock_s"], 0.06)


def test_stream_stats_window_is_incremental() -> None:
    stats = probe.StreamStats("clock")
    stats.record(1_000_000_000, 10.0)
    stats.record(1_100_000_000, 10.1)

    first = stats.take_window(0.2, 1_150_000_000)
    second = stats.take_window(1.0, 1_150_000_000)

    assert first["received"] == 2
    assert math.isclose(first["wall_hz"], 10.0)
    assert math.isclose(first["age_vs_clock_s"], 0.05)
    assert second["received"] == 0


def test_tf_extrapolation_parser_preserves_requested_latest_delta() -> None:
    message = (
        "Lookup would require extrapolation into the future. "
        "Requested time 6.720000 but the latest data is at time 5.649000"
    )
    match = probe.EXTRAPOLATION.search(message)
    assert match is not None
    assert math.isclose(float(match.group(1)) - float(match.group(2)), 1.071)


def test_phase1_sidecar_is_attached_to_representative_runtime_smokes() -> None:
    scripts = (
        "smoke_navigation_core_sim.sh",
        "smoke_navigation_canonical_sim.sh",
        "smoke_navigation_no_obstacles_sim.sh",
        "smoke_navigation_snapshot.sh",
        "smoke_web_cockpit.sh",
    )
    for script in scripts:
        source = (ROOT / "tools" / script).read_text(encoding="utf-8")
        assert "smoke_start_runtime_timing_probe" in source

    harness = (ROOT / "tools" / "smoke_harness.sh").read_text(encoding="utf-8")
    assert "runtime_timing_probe.py" in harness
    assert "runtime_timing.json" in harness


def test_snapshot_and_web_probes_record_request_windows() -> None:
    snapshot_path = ROOT / "tools" / "smoke_navigation_snapshot.py"
    web_path = ROOT / "tools" / "smoke_web_cockpit.py"
    snapshot = snapshot_path.read_text(encoding="utf-8")
    web = web_path.read_text(encoding="utf-8")

    compile(snapshot, str(snapshot_path), "exec")
    compile(web, str(web_path), "exec")
    assert '"snapshot_request_timings"' in snapshot
    assert '"started_monotonic_s"' in snapshot
    assert '"request_timings"' in web
    assert '"operation": operation' in web
    assert "scenario(evidence)" in web
