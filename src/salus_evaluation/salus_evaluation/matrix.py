"""Pure, reproducible speed-by-geometry matrix definitions and summaries."""

from __future__ import annotations

import csv
import html
import json
import math
import re
from dataclasses import dataclass
from pathlib import Path

import yaml


SCHEMA_VERSION = 1
EFFECTIVE_SPEED_TOLERANCE_MPS = 1.0e-6


def parse_effective_speed(readback):
    """Extract the one numeric value from a ROS parameter readback, if unambiguous."""
    values = re.findall(
        r"(?<![\w.])[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?(?![\w.])",
        str(readback),
    )
    if len(values) != 1:
        return None
    value = float(values[0])
    return value if math.isfinite(value) else None


def effective_speed_matches(requested_mps, readback, tolerance_mps=EFFECTIVE_SPEED_TOLERANCE_MPS):
    """Return the parsed effective speed and whether it matches the request."""
    requested = _finite_positive(requested_mps, "requested_speed_mps")
    if not math.isfinite(tolerance_mps) or tolerance_mps < 0.0:
        raise ValueError("speed tolerance must be finite and non-negative")
    effective = parse_effective_speed(readback)
    return effective, effective is not None and abs(effective - requested) <= tolerance_mps


def matrix_exit_code(trial_outcomes):
    """Fail only setup or existing functional-gate failures after all trials ran."""
    return int(any(
        outcome in ("setup_failure", "functional_failure")
        for outcome in trial_outcomes
    ))


@dataclass(frozen=True)
class MatrixCase:
    """One requested path geometry, distinct from the plan Nav2 produces."""

    case_id: str
    scenario: str
    direction: str
    requested_radius_m: float | None


@dataclass(frozen=True)
class MatrixCell:
    """One speed/geometry/repetition trial with a stable filesystem identifier."""

    matrix_id: str
    case: MatrixCase
    speed_mps: float
    repetition: int

    @property
    def trial_id(self) -> str:
        radius = "straight" if self.case.requested_radius_m is None else (
            f"r{self.case.requested_radius_m:g}".replace(".", "p")
        )
        speed = f"v{self.speed_mps:g}".replace(".", "p")
        return (
            f"{self.case.case_id}-{self.case.direction}-{radius}-{speed}"
            f"-rep{self.repetition:02d}"
        )


def _require_keys(value, required):
    if not isinstance(value, dict) or set(value) != set(required):
        raise ValueError(f"invalid keys; expected={sorted(required)}")


def _finite_positive(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite positive number")
    result = float(value)
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return result


def load_matrix(path):
    """Load a strict matrix definition without silently accepting new semantics."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    _require_keys(raw, ("schema_version", "id", "repetitions", "max_speed_mps",
                        "speeds_mps", "cases"))
    if raw["schema_version"] != SCHEMA_VERSION:
        raise ValueError("unsupported matrix schema_version")
    matrix_id = str(raw["id"]).strip()
    if not matrix_id:
        raise ValueError("matrix id must be non-empty")
    if not isinstance(raw["repetitions"], int) or raw["repetitions"] <= 0:
        raise ValueError("repetitions must be a positive integer")
    maximum = _finite_positive(raw["max_speed_mps"], "max_speed_mps")
    if not isinstance(raw["speeds_mps"], list) or not raw["speeds_mps"]:
        raise ValueError("speeds_mps must be a non-empty list")
    speeds = tuple(_finite_positive(item, "speed_mps") for item in raw["speeds_mps"])
    if len(set(speeds)) != len(speeds) or any(item > maximum for item in speeds):
        raise ValueError("speeds_mps must be unique and no greater than max_speed_mps")
    if not isinstance(raw["cases"], list) or not raw["cases"]:
        raise ValueError("cases must be a non-empty list")
    cases = []
    for item in raw["cases"]:
        _require_keys(item, ("id", "scenario", "direction", "requested_radius_m"))
        case_id, scenario, direction = (str(item["id"]).strip(),
                                        str(item["scenario"]).strip(),
                                        str(item["direction"]).strip())
        if not case_id or not scenario or direction not in ("left", "right", "straight"):
            raise ValueError("case needs id, scenario and left/right/straight direction")
        radius = item["requested_radius_m"]
        if direction == "straight":
            if radius is not None:
                raise ValueError("straight case requested_radius_m must be null")
        else:
            radius = _finite_positive(radius, "requested_radius_m")
        cases.append(MatrixCase(case_id, scenario, direction, radius))
    if len({case.case_id for case in cases}) != len(cases):
        raise ValueError("case ids must be unique")
    return matrix_id, int(raw["repetitions"]), maximum, speeds, tuple(cases)


def expand_matrix(path):
    """Expand a matrix in deterministic case, speed, then repetition order."""
    matrix_id, repetitions, _maximum, speeds, cases = load_matrix(path)
    return tuple(MatrixCell(matrix_id, case, speed, repetition)
                 for case in cases for speed in speeds
                 for repetition in range(1, repetitions + 1))


def _percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    low, high = math.floor(index), math.ceil(index)
    return ordered[low] + (ordered[high] - ordered[low]) * (index - low)


def continuous_summary(values):
    """Aggregate finite samples; P95 is deliberately unavailable below two."""
    values = [
        float(value) for value in values
        if value is not None and math.isfinite(float(value))
    ]
    if not values:
        return {"count": 0, "min": None, "median": None, "p95": None, "max": None}
    return {"count": len(values), "min": min(values), "median": _percentile(values, .5),
            "p95": _percentile(values, .95) if len(values) >= 2 else None, "max": max(values)}


def aggregate_trials(cells, trial_summaries):
    """Summarize individual current-format bundles by speed and requested geometry."""
    by_id = {cell.trial_id: cell for cell in cells}
    groups = {}
    for trial_id, summary in trial_summaries.items():
        cell = by_id[trial_id]
        key = (cell.case.case_id, cell.speed_mps)
        groups.setdefault(key, []).append((cell, summary))
    result = []
    for key in sorted(groups):
        entries = groups[key]
        case, _ = entries[0][0].case, entries[0][0].speed_mps
        successes = [item for _cell, item in entries if item.get("terminal_status") == 4]

        def values(*keys):
            found = []
            for _cell, item in entries:
                current = item
                for name in keys:
                    current = current.get(name) if isinstance(current, dict) else None
                found.append(current)
            return found
        result.append({
            "case_id": case.case_id, "direction": case.direction,
            "requested_radius_m": case.requested_radius_m,
            "requested_curvature_per_m": (
                None if case.requested_radius_m is None
                else 1.0 / case.requested_radius_m
            ),
            "speed_mps": entries[0][0].speed_mps, "trial_count": len(entries),
            "success_count": len(successes), "failure_count": len(entries) - len(successes),
            "success_rate": len(successes) / len(entries),
            "cross_track_rmse_m": continuous_summary(values("metrics", "cross_track_rms_m")),
            "cross_track_p95_m": continuous_summary(values("metrics", "cross_track_p95_m")),
            "heading_p95_rad": continuous_summary(values("metrics", "heading_p95_rad")),
            "final_xy_error_m": continuous_summary(values("arrival", "final_distance_m")),
            "overshoot_m": continuous_summary(values("arrival", "overshoot_m")),
            "replans": continuous_summary(values("replans")),
            "steering_saturation_intervals": continuous_summary(
                values("command_chain", "steering_saturation", "interval_count")
            ),
            "steering_requested_to_applied_rad": continuous_summary(values(
                "command_chain", "ackermann",
                "requested_to_applied_steer_delta_rad", "max",
            )),
            "trial_ids": [cell.trial_id for cell, _summary in entries],
            "performance_gate_state": "calibrating",
        })
    return result


def write_matrix_artifacts(output_dir, matrix_path, cells, trial_dirs):
    """Write deterministic JSON/CSV/HTML matrix artifacts from individual bundles."""
    summaries = {}
    for cell, trial_dir in zip(cells, trial_dirs):
        summary_path = Path(trial_dir) / "summary.json"
        summaries[cell.trial_id] = json.loads(summary_path.read_text(encoding="utf-8"))
    rows = aggregate_trials(cells, summaries)
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    manifest = {"schema_version": SCHEMA_VERSION, "matrix": str(matrix_path),
                "trials": [{"trial_id": cell.trial_id, "artifact_dir": str(directory)}
                           for cell, directory in zip(cells, trial_dirs)],
                "performance_gates": "calibrating/report-only"}
    (root / "matrix-manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )
    (root / "matrix-summary.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True) + "\n"
    )
    flat = [{key: (json.dumps(value, sort_keys=True) if isinstance(value, (dict, list)) else value)
             for key, value in row.items()} for row in rows]
    with (root / "matrix-summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=sorted(flat[0]) if flat else [])
        writer.writeheader()
        writer.writerows(flat)
    rendered = html.escape(json.dumps(rows, indent=2, sort_keys=True))
    (root / "matrix-report.html").write_text(
        "<!doctype html><meta charset=utf-8><title>SALUS navigation matrix</title>"
        "<h1>SALUS navigation matrix (calibrating)</h1><pre>" + rendered + "</pre>",
        encoding="utf-8")
    return root
