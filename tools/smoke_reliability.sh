#!/usr/bin/env bash
# Daily-only repetition. PR CI runs each deterministic scenario once.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

scenarios=(
  smoke_control_sim.sh smoke_motion_sim.sh smoke_localization_sim.sh
  smoke_lidar_sim.sh smoke_safety_sim.sh smoke_integration_sim.sh smoke_navigation_core_sim.sh
  smoke_navigation_zones_sim.sh
)
for attempt in 1 2 3; do
  echo "Smoke reliability pass ${attempt}/3"
  for scenario in "${scenarios[@]}"; do
    "${repo_dir}/tools/${scenario}"
  done
done
