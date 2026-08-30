#!/usr/bin/env python3
"""Run one registered smoke scenario using registry-owned execution metadata."""
from __future__ import annotations

import argparse
import os
import subprocess
from pathlib import Path

try:
    from tools.smoke_registry import BY_ID
except ModuleNotFoundError:
    from smoke_registry import BY_ID

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario_id")
    parser.add_argument("--context", choices=("ci", "nightly", "manual"), default="ci")
    args = parser.parse_args()

    scenario = BY_ID.get(args.scenario_id)
    if scenario is None:
        parser.error(f"unknown smoke scenario: {args.scenario_id}")

    participation = scenario["participation"]
    if args.context == "ci" and not participation["pr"]:
        parser.error(f"{args.scenario_id} does not participate in PR/main CI")
    if args.context == "nightly" and not participation["nightly"]:
        parser.error(f"{args.scenario_id} does not participate in nightly")

    # Manual execution preserves registry-owned metadata without making a
    # resource-heavy scenario a required PR/main gate. Manual runs use the CI
    # hard-timeout budget and may execute any registered scenario.
    timeout_context = "nightly" if args.context == "nightly" else "ci"
    env = os.environ.copy()
    env["SMOKE_HARD_TIMEOUT_S"] = str(scenario["timeouts_s"][timeout_context])
    for key, value in scenario.get("env", {}).items():
        env[str(key)] = str(value)

    script = ROOT / scenario["script"]
    command = [str(ROOT / "tools" / "run_smoke.sh"), str(script)]
    print(
        f"[registered-smoke] id={args.scenario_id} context={args.context} "
        f"timeout_s={env['SMOKE_HARD_TIMEOUT_S']} script={scenario['script']}",
        flush=True,
    )
    return subprocess.call(command, cwd=ROOT, env=env)


if __name__ == "__main__":
    raise SystemExit(main())
