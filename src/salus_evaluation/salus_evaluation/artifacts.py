"""Versioned, dependency-free evaluation artifact persistence."""

import csv
import html
import json
import math
from dataclasses import asdict, is_dataclass
from pathlib import Path


def _value(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if is_dataclass(value):
        return _value(asdict(value))
    if isinstance(value, dict):
        return {key: _value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_value(item) for item in value]
    if hasattr(value, "value"):
        return value.value
    return value


def write_artifacts(output_dir, manifest, summary, streams):
    """Write the contractual JSON/CSV/HTML bundle, even for failed trials."""
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=True)
    for name, value in (("manifest.json", manifest), ("summary.json", summary)):
        (root / name).write_text(json.dumps(_value(value), indent=2, sort_keys=True) + "\n",
                                 encoding="utf-8")
    for stream_name, rows in streams.items():
        rows = [_value(row) for row in rows]
        keys = sorted({key for row in rows for key in row}) if rows else []
        with (root / f"{stream_name}.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)
    rendered = html.escape(json.dumps(_value(summary), indent=2, sort_keys=True))
    (root / "report.html").write_text(
        "<!doctype html><meta charset=utf-8><title>SALUS navigation evaluation</title>"
        "<h1>SALUS navigation evaluation</h1><pre>" + rendered + "</pre>", encoding="utf-8")
    return root
