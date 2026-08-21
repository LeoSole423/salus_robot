#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-53}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-patrol-battery-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init patrol-battery-return
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch patrol_battery "ros2 launch salus_bringup integration_sim.launch.py launch_routes:=true launch_patrol:=true world:=${free_world} patrol_battery_guard_topic:=/smoke/battery_mission_guard patrol_battery_state_topic:=/smoke/battery_state"
  smoke_run patrol_battery_probe "python3 /ros2_ws/tools/smoke_patrol_battery_sim.py"
  smoke_note "patrol_battery_return_valid"
'
