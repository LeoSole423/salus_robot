#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-47}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-snapshot-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init navigation-snapshot
  trap smoke_cleanup EXIT
  obstacle_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/empty.world"
  smoke_start_launch integration "ros2 launch salus_bringup integration_sim.launch.py world:=${obstacle_world} capability_profile:=no_obstacle_detection"
  smoke_start_launch snapshot "ros2 launch salus_navigation navigation_snapshot_sim.launch.py"
  smoke_run snapshot_probe "python3 /ros2_ws/tools/smoke_navigation_snapshot.py"
  smoke_note "snapshot_contract_valid"
'
