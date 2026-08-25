#!/usr/bin/env python3
"""Validate repository conventions without requiring a ROS installation."""

import ast
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
EXPECTED = {
    "salus_interfaces", "salus_description", "salus_hardware",
    "salus_localization", "salus_perception", "salus_control",
    "salus_navigation", "salus_navigation_bt", "salus_web",
    "salus_simulation", "salus_bringup",
    "salus_evaluation",
}


def main() -> int:
    errors = []
    packages = {}
    for manifest in sorted((ROOT / "src").glob("*/package.xml")):
        name = ET.parse(manifest).getroot().findtext("name", default="")
        packages[name] = manifest.parent
        if name != manifest.parent.name:
            errors.append(f"{manifest}: package name does not match directory")
        if not (manifest.parent / "README.md").is_file():
            errors.append(f"{manifest.parent}: missing README.md")
    if set(packages) != EXPECTED:
        errors.append(f"package set differs: expected={sorted(EXPECTED)}, actual={sorted(packages)}")

    ignored_trees = {".git", "build", "install", "log"}
    for document in sorted(ROOT.rglob("*.md")):
        relative_parts = document.relative_to(ROOT).parts
        if ignored_trees.intersection(relative_parts):
            continue
        text = document.read_text(encoding="utf-8")
        for target in re.findall(r"\[[^]]+\]\(([^)]+)\)", text):
            if target.startswith(("http://", "https://", "#")):
                continue
            path = target.split("#", 1)[0]
            if path and not (document.parent / path).resolve().exists():
                errors.append(f"{document}: broken link {target}")
        if re.search(r"/home/[^/]+/", text):
            errors.append(f"{document}: contains an absolute home path")

    # Smoke probes commonly need both filesystem paths and ROS Path messages.
    # Detect duplicate bindings early: Python silently lets the latter import
    # replace the former, which otherwise only surfaces during CI execution.
    for probe in sorted((ROOT / "tools").glob("smoke_*_sim.py")):
        bindings: dict[str, int] = {}
        tree = ast.parse(probe.read_text(encoding="utf-8"), filename=str(probe))
        for statement in tree.body:
            if isinstance(statement, ast.Import):
                names = [alias.asname or alias.name.split(".")[0] for alias in statement.names]
            elif isinstance(statement, ast.ImportFrom):
                names = [alias.asname or alias.name for alias in statement.names]
            else:
                continue
            for name in names:
                bindings[name] = bindings.get(name, 0) + 1
        for name, count in sorted(bindings.items()):
            if count > 1:
                errors.append(f"{probe}: duplicate imported binding {name!r}")

    if errors:
        print("Repository validation failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print(f"Repository validation passed: {len(packages)} packages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
