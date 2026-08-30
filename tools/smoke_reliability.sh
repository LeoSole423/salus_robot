#!/usr/bin/env bash
# Daily-only repetition. PR CI runs each deterministic scenario once.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

mapfile -t scenarios < <(python3 tools/smoke_registry.py --nightly-scripts)
repetitions="${SMOKE_REPETITIONS:-$(python3 tools/smoke_registry.py --nightly-repetitions)}"
failures=0
scenario_runs=0
passed_runs=0
completed_suites=0
summary_path="${SMOKE_RELIABILITY_SUMMARY:-${repo_dir}/artifacts/smokes/reliability-summary.json}"

write_summary() {
  local status="$1"
  mkdir -p "$(dirname "${summary_path}")"
  python3 - "${summary_path}" "${status}" "${repetitions}" "${#scenarios[@]}" \
    "${completed_suites}" "${scenario_runs}" "${passed_runs}" "${failures}" <<'PY'
import json
import os
import sys
from pathlib import Path

(
    destination,
    status,
    configured_suites,
    scenarios_per_suite,
    completed_suites,
    completed_scenarios,
    passed_scenarios,
    failed_scenarios,
) = sys.argv[1:]
payload = {
    "status": status,
    "configured_suites": int(configured_suites),
    "scenarios_per_suite": int(scenarios_per_suite),
    "expected_scenarios": int(configured_suites) * int(scenarios_per_suite),
    "completed_suites": int(completed_suites),
    "completed_scenarios": int(completed_scenarios),
    "passed_scenarios": int(passed_scenarios),
    "failed_scenarios": int(failed_scenarios),
}
path = Path(destination)
temporary = path.with_suffix(path.suffix + ".tmp")
temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
os.replace(temporary, path)
PY
}

write_summary running
for ((attempt = 1; attempt <= repetitions; attempt++)); do
  echo "Smoke reliability pass ${attempt}/${repetitions}"
  for scenario in "${scenarios[@]}"; do
    scenario_runs=$((scenario_runs + 1))
    if "${repo_dir}/tools/run_smoke.sh" "${repo_dir}/tools/${scenario}"; then
      passed_runs=$((passed_runs + 1))
    else
      failures=$((failures + 1))
      printf 'FAILED: pass=%s scenario=%s\n' "${attempt}" "${scenario}" >&2
    fi
    write_summary running
  done
  completed_suites="${attempt}"
  write_summary running
done
if ((failures > 0)); then
  write_summary failed
  printf 'Smoke reliability failed: %s scenario runs failed\n' "${failures}" >&2
  exit 1
fi
write_summary passed
printf 'Smoke reliability passed: %s/%s suites, %s/%s scenarios\n' \
  "${completed_suites}" "${repetitions}" "${passed_runs}" "${scenario_runs}"
