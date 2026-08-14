#!/usr/bin/env bash
# Daily-only repetition. PR CI runs each deterministic scenario once.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

scenarios=(
  smoke_control_sim.sh smoke_motion_sim.sh smoke_localization_sim.sh
  smoke_lidar_sim.sh smoke_safety_sim.sh smoke_integration_sim.sh smoke_navigation_core_sim.sh
  smoke_navigation_zones_sim.sh smoke_route_executor_sim.sh
)
repetitions="${SMOKE_REPETITIONS:-3}"
failures=0
for ((attempt = 1; attempt <= repetitions; attempt++)); do
  echo "Smoke reliability pass ${attempt}/${repetitions}"
  for scenario in "${scenarios[@]}"; do
    if ! "${repo_dir}/tools/run_smoke.sh" "${repo_dir}/tools/${scenario}"; then
      failures=$((failures + 1))
      printf 'FAILED: pass=%s scenario=%s\n' "${attempt}" "${scenario}" >&2
    fi
  done
done
if ((failures > 0)); then
  printf 'Smoke reliability failed: %s scenario runs failed\n' "${failures}" >&2
  exit 1
fi
