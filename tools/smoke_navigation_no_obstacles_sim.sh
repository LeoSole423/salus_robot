#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-44}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-navigation-no-obstacles-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init navigation-no-obstacles-free-world
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch navigation_no_obstacles "ros2 launch salus_bringup integration_sim.launch.py world:=${free_world} capability_profile:=no_obstacle_detection"
  smoke_run navigation_no_obstacles_core "EXPECT_NO_OBSTACLE_DETECTION=1 python3 /ros2_ws/tools/smoke_navigation_core_sim.py"
  smoke_run navigation_no_obstacles_zones "python3 /ros2_ws/tools/smoke_navigation_zones_sim.py"
  smoke_note "nav2_operates_without_fictitious_lidar_and_preserves_safe_command_authority"
'
