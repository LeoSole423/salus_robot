#!/usr/bin/env python3
"""Conservative change-aware selector for SALUS pull-request smoke tests."""

from __future__ import annotations

import argparse
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

try:
    from tools.smoke_registry import ids as registry_ids
except ModuleNotFoundError:  # Direct execution: python3 tools/ci_select_smokes.py
    from smoke_registry import ids as registry_ids

CORE_SMOKES = registry_ids(family="core", participation="pr")
NAVIGATION_SMOKES = registry_ids(family="navigation", participation="pr")
ALL_SMOKES = registry_ids(participation="pr")

FULL_PREFIXES = (
    ".github/workflows/",
    "tools/",
    "src/salus_interfaces/",
    "src/salus_bringup/",
    "src/salus_description/",
    "src/salus_simulation/",
    "src/salus_hardware/",
)
FULL_FILES = {
    "Dockerfile",
    "compose.yaml",
    "dependencies.repos",
    "entrypoint.sh",
    "docs/package-map.yaml",
}
FAST_ONLY_PREFIXES = ("docs/",)
FAST_ONLY_FILES = {
    ".editorconfig",
    ".gitignore",
    "AGENTS.md",
    "LICENSE",
    "README.md",
}

PACKAGE_SMOKES = {
    "src/salus_control/": {
        "control",
        "motion",
        "safety",
        "integration",
    },
    "src/salus_localization/": {
        "localization",
        "localization_canonical",
        "sensor_selection",
        "integration",
        "navigation",
        "navigation_canonical",
    },
    "src/salus_navigation/": {
        "safety",
        "integration",
        "navigation",
        "navigation_canonical",
        "navigation_no_obstacles",
        "zones",
        "routes",
        "patrol_battery",
        "snapshot",
    },
    "src/salus_navigation_bt/": {
        "integration",
        "navigation",
        "navigation_canonical",
        "navigation_no_obstacles",
        "zones",
        "routes",
        "patrol_battery",
        "snapshot",
    },
    "src/salus_perception/": {
        "lidar",
        "integration",
        "navigation",
        "navigation_canonical",
    },
    "src/salus_web/": {
        "integration",
    },
    # No current runtime smoke owns salus_evaluation. Its unit/lint/build coverage
    # remains in the mandatory fast gate.
    "src/salus_evaluation/": set(),
}


@dataclass(frozen=True)
class Selection:
    classification: str
    full_ci: bool
    smokes: frozenset[str]
    changed_files: tuple[str, ...]
    reasons: tuple[str, ...]

    @property
    def run_simulation_core(self) -> bool:
        return bool(self.smokes.intersection(CORE_SMOKES))

    @property
    def run_navigation_missions(self) -> bool:
        return bool(self.smokes.intersection(NAVIGATION_SMOKES))


def _clean_paths(paths: Iterable[str]) -> tuple[str, ...]:
    cleaned = []
    for raw in paths:
        path = raw.strip().replace("\\", "/")
        while path.startswith("./"):
            path = path[2:]
        if path:
            cleaned.append(path)
    return tuple(sorted(set(cleaned)))


def classify(paths: Iterable[str], *, force_full_reason: str | None = None) -> Selection:
    changed = _clean_paths(paths)

    if force_full_reason:
        return Selection(
            classification="FULL",
            full_ci=True,
            smokes=frozenset(ALL_SMOKES),
            changed_files=changed,
            reasons=(force_full_reason,),
        )

    if not changed:
        return Selection(
            classification="FULL",
            full_ci=True,
            smokes=frozenset(ALL_SMOKES),
            changed_files=changed,
            reasons=("no changed files were available; conservative FULL CI fallback",),
        )

    selected: set[str] = set()
    reasons: list[str] = []
    unknown: list[str] = []

    for path in changed:
        if path in FULL_FILES or any(path.startswith(prefix) for prefix in FULL_PREFIXES):
            reasons.append(f"{path}: shared/structural boundary requires FULL CI")
            return Selection(
                classification="FULL",
                full_ci=True,
                smokes=frozenset(ALL_SMOKES),
                changed_files=changed,
                reasons=tuple(reasons),
            )

        if path in FAST_ONLY_FILES or any(path.startswith(prefix) for prefix in FAST_ONLY_PREFIXES):
            reasons.append(f"{path}: fast-gate-only metadata/documentation")
            continue

        matched = False
        for prefix, smokes in PACKAGE_SMOKES.items():
            if path.startswith(prefix):
                matched = True
                selected.update(smokes)
                if smokes:
                    reasons.append(
                        f"{path}: {prefix.rstrip('/').split('/')[-1]} -> "
                        + ", ".join(sorted(smokes))
                    )
                else:
                    reasons.append(
                        f"{path}: salus_evaluation has no owned runtime smoke; "
                        "fast gate covers build/lint/unit tests"
                    )
                break
        if not matched:
            unknown.append(path)

    if unknown:
        reasons.extend(f"{path}: unclassified path -> FULL CI fallback" for path in unknown)
        return Selection(
            classification="FULL",
            full_ci=True,
            smokes=frozenset(ALL_SMOKES),
            changed_files=changed,
            reasons=tuple(reasons),
        )

    if selected:
        owners = sorted(
            {
                prefix.rstrip("/").split("/")[-1]
                for path in changed
                for prefix in PACKAGE_SMOKES
                if path.startswith(prefix)
            }
        )
        return Selection(
            classification="TARGETED:" + ",".join(owners),
            full_ci=False,
            smokes=frozenset(selected),
            changed_files=changed,
            reasons=tuple(reasons),
        )

    return Selection(
        classification="FAST_GATE_ONLY",
        full_ci=False,
        smokes=frozenset(),
        changed_files=changed,
        reasons=tuple(reasons),
    )


def outputs(selection: Selection) -> dict[str, str]:
    data = {
        "classification": selection.classification,
        "full_ci": str(selection.full_ci).lower(),
        "run_simulation_core": str(selection.run_simulation_core).lower(),
        "run_navigation_missions": str(selection.run_navigation_missions).lower(),
        "run_smokes": str(bool(selection.smokes)).lower(),
        "smoke_matrix": json.dumps(
            {"include": [{"id": smoke} for smoke in ALL_SMOKES if smoke in selection.smokes]},
            separators=(",", ":"),
        ),
    }
    for smoke in ALL_SMOKES:
        data[f"smoke_{smoke}"] = str(smoke in selection.smokes).lower()
    return data


def _write_github_output(path: str, selection: Selection) -> None:
    with open(path, "a", encoding="utf-8") as handle:
        for key, value in outputs(selection).items():
            handle.write(f"{key}={value}\n")


def _write_summary(path: str, selection: Selection) -> None:
    selected = [name for name in ALL_SMOKES if name in selection.smokes]
    skipped = [name for name in ALL_SMOKES if name not in selection.smokes]
    with open(path, "a", encoding="utf-8") as handle:
        handle.write("## Change-aware CI selection\n\n")
        handle.write(f"- Classification: {selection.classification}\n")
        handle.write(f"- FULL CI fallback: {str(selection.full_ci).lower()}\n")
        handle.write(
            "- Jobs: build-unit always; "
            + ("simulation-core " if selection.run_simulation_core else "simulation-core skipped ")
            + (f"{len(selection.smokes)} isolated smoke job(s)" if selection.smokes else "smoke matrix skipped")
            + "\n"
        )
        handle.write("- Selected smokes: " + (", ".join(selected) or "_none_") + "\n")
        handle.write("- Skipped smokes: " + (", ".join(skipped) or "_none_") + "\n\n")
        handle.write("### Changed files\n\n")
        if selection.changed_files:
            for changed in selection.changed_files:
                handle.write(f"- {changed}\n")
        else:
            handle.write("- _none available_\n")
        handle.write("\n### Reasons\n\n")
        for reason in selection.reasons:
            handle.write(f"- {reason}\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--files", type=Path, help="newline-separated changed-file list")
    parser.add_argument("--full-reason", help="force FULL CI with this reason")
    parser.add_argument("--github-output", default=os.environ.get("GITHUB_OUTPUT"))
    parser.add_argument("--github-step-summary", default=os.environ.get("GITHUB_STEP_SUMMARY"))
    parser.add_argument("--json", action="store_true", help="print selection as JSON")
    args = parser.parse_args()

    paths: list[str] = []
    if args.files:
        paths = args.files.read_text(encoding="utf-8").splitlines()

    selection = classify(paths, force_full_reason=args.full_reason)

    print(f"[ci-selector] classification={selection.classification}")
    print(f"[ci-selector] full_ci={str(selection.full_ci).lower()}")
    print("[ci-selector] changed files:")
    for path in selection.changed_files or ("<none>",):
        print(f"  - {path}")
    print("[ci-selector] selected smokes:")
    selected = [name for name in ALL_SMOKES if name in selection.smokes]
    for name in selected or ["<none>"]:
        print(f"  - {name}")
    print("[ci-selector] skipped smokes:")
    skipped = [name for name in ALL_SMOKES if name not in selection.smokes]
    for name in skipped or ["<none>"]:
        print(f"  - {name}")
    print("[ci-selector] reasons:")
    for reason in selection.reasons:
        print(f"  - {reason}")

    if args.github_output:
        _write_github_output(args.github_output, selection)
    if args.github_step_summary:
        _write_summary(args.github_step_summary, selection)
    if args.json:
        print(
            json.dumps(
                {
                    **outputs(selection),
                    "changed_files": selection.changed_files,
                    "reasons": selection.reasons,
                    "selected_smokes": selected,
                    "skipped_smokes": skipped,
                },
                indent=2,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
