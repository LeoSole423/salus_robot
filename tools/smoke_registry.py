#!/usr/bin/env python3
"""Read and validate the authoritative SALUS smoke scenario registry."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
REGISTRY_PATH = ROOT / "tools" / "smoke_scenarios.json"


def load_registry(path: Path = REGISTRY_PATH) -> tuple[dict[str, Any], ...]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if payload.get("schema_version") != 1:
        raise ValueError("unsupported smoke registry schema")
    scenarios = tuple(payload.get("scenarios", ()))
    validate_registry(scenarios)
    return scenarios


def validate_registry(scenarios: tuple[dict[str, Any], ...]) -> None:
    ids: set[str] = set()
    for scenario in scenarios:
        scenario_id = str(scenario.get("id", ""))
        if not scenario_id or scenario_id in ids:
            raise ValueError(f"invalid or duplicate scenario id: {scenario_id!r}")
        ids.add(scenario_id)
        script = ROOT / str(scenario.get("script", ""))
        if not script.is_file():
            raise ValueError(f"{scenario_id}: smoke script does not exist: {script}")
        if scenario.get("family") not in {"core", "navigation", "operational"}:
            raise ValueError(f"{scenario_id}: invalid family")
        participation = scenario.get("participation", {})
        for context in ("pr", "full", "main", "nightly"):
            if not isinstance(participation.get(context), bool):
                raise ValueError(f"{scenario_id}: participation.{context} must be boolean")
        timeouts = scenario.get("timeouts_s", {})
        for context in ("ci", "nightly"):
            if not isinstance(timeouts.get(context), int) or timeouts[context] <= 0:
                raise ValueError(f"{scenario_id}: timeouts_s.{context} must be positive")
        repetitions = scenario.get("nightly_repetitions")
        if not isinstance(repetitions, int) or repetitions < 0:
            raise ValueError(f"{scenario_id}: nightly_repetitions must be non-negative")
        if participation["nightly"] != (repetitions > 0):
            raise ValueError(f"{scenario_id}: nightly participation/repetitions disagree")


SCENARIOS = load_registry()
BY_ID = {scenario["id"]: scenario for scenario in SCENARIOS}


def ids(*, family: str | None = None, participation: str | None = None) -> tuple[str, ...]:
    selected = []
    for scenario in SCENARIOS:
        if family is not None and scenario["family"] != family:
            continue
        if participation is not None and not scenario["participation"][participation]:
            continue
        selected.append(scenario["id"])
    return tuple(selected)


def nightly_scripts() -> tuple[str, ...]:
    return tuple(
        Path(scenario["script"]).name
        for scenario in SCENARIOS
        if scenario["participation"]["nightly"]
    )


def default_nightly_repetitions() -> int:
    repetitions = {
        scenario["nightly_repetitions"]
        for scenario in SCENARIOS
        if scenario["participation"]["nightly"]
    }
    if len(repetitions) != 1:
        raise ValueError("current nightly runner requires a uniform repetition count")
    return repetitions.pop()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nightly-scripts", action="store_true")
    parser.add_argument("--nightly-repetitions", action="store_true")
    parser.add_argument("--ids", choices=("pr", "full", "main", "nightly"))
    args = parser.parse_args()
    if args.nightly_scripts:
        print("\n".join(nightly_scripts()))
    elif args.nightly_repetitions:
        print(default_nightly_repetitions())
    elif args.ids:
        print("\n".join(ids(participation=args.ids)))
    else:
        print(json.dumps(SCENARIOS, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
