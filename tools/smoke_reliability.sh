#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

scenario_id="${SMOKE_SCENARIO_ID:?SMOKE_SCENARIO_ID is required}"
repetitions="${SMOKE_REPETITIONS:?SMOKE_REPETITIONS is required}"
summary_path="${SMOKE_RELIABILITY_SUMMARY:-${repo_dir}/artifacts/smokes/reliability-${scenario_id}.json}"
sensor_profile="${SMOKE_SENSOR_PROFILE:-clean}"
sensor_seed_base="${SMOKE_SENSOR_SEED_BASE:-${SMOKE_SENSOR_SEED:-6400}}"
if [[ ! "${sensor_seed_base}" =~ ^[0-9]+$ ]]; then
  echo "SMOKE_SENSOR_SEED_BASE must be a non-negative integer" >&2
  exit 2
fi

python3 - "${scenario_id}" <<'PY'
import sys
from tools.smoke_registry import BY_ID
scenario = BY_ID.get(sys.argv[1])
if scenario is None:
    raise SystemExit(f"unknown smoke scenario: {sys.argv[1]}")
if not scenario["participation"]["nightly"]:
    raise SystemExit(f"scenario is not nightly-enabled: {sys.argv[1]}")
PY

completed=0
passed=0
failed=0
sensor_seed_history=""

write_summary() {
  local status="$1"
  mkdir -p "$(dirname "${summary_path}")"
  python3 - "${summary_path}" "${scenario_id}" "${status}" "${repetitions}" \
    "${completed}" "${passed}" "${failed}" "${sensor_profile}" \
    "${sensor_seed_base}" "${sensor_seed_history}" <<'PY'
import json
import os
import sys
from pathlib import Path

(destination, scenario_id, status, configured, completed, passed, failed, profile, seed_base, seed_history) = sys.argv[1:]
configured_i = int(configured)
completed_i = int(completed)
payload = {
    "scenario_id": scenario_id,
    "status": status,
    "configured_repetitions": configured_i,
    "completed_repetitions": completed_i,
    "passed_repetitions": int(passed),
    "failed_repetitions": int(failed),
    "incomplete_repetitions": max(0, configured_i - completed_i),
    "simulation_sensors": {
        "profile": profile,
        "seed_base": int(seed_base),
        "seeds_by_repetition": [int(value) for value in seed_history.split()],
    },
}
path = Path(destination)
temporary = path.with_suffix(path.suffix + ".tmp")
temporary.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
os.replace(temporary, path)
PY
}

write_summary running
for ((attempt = 1; attempt <= repetitions; attempt++)); do
  echo "[nightly] scenario=${scenario_id} repetition=${attempt}/${repetitions}"
  sensor_seed=$((sensor_seed_base + attempt - 1))
  sensor_seed_history+=" ${sensor_seed}"
  if SMOKE_SENSOR_PROFILE="${sensor_profile}" SMOKE_SENSOR_SEED="${sensor_seed}" \
      python3 tools/run_registered_smoke.py "${scenario_id}" --context nightly; then
    passed=$((passed + 1))
  else
    failed=$((failed + 1))
    printf 'FAILED: scenario=%s repetition=%s\n' "${scenario_id}" "${attempt}" >&2
  fi
  completed=$((completed + 1))
  write_summary running
done

if ((failed > 0)); then
  write_summary failed
  printf 'Nightly reliability failed: scenario=%s passed=%s failed=%s\n' \
    "${scenario_id}" "${passed}" "${failed}" >&2
  exit 1
fi

write_summary passed
printf 'Nightly reliability passed: scenario=%s %s/%s\n' \
  "${scenario_id}" "${passed}" "${repetitions}"
