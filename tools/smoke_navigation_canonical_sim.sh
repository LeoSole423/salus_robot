#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-44}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-navigation-canonical-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init navigation-canonical-free-world
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch navigation_canonical "ros2 launch salus_bringup integration_sim.launch.py world:=${free_world} command_input_mode:=canonical_vehicle_command"
  smoke_run navigation_canonical_core "EXPECT_CANONICAL_COMMAND=1 python3 /ros2_ws/tools/smoke_navigation_core_sim.py"
  smoke_note "nav2_goal_traverses_fresh_canonical_vehicle_command"
'
