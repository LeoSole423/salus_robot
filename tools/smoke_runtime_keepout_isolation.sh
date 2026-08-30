#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

run_variant() {
  local variant="$1"
  local ros_domain="$2"
  local launch_args="$3"

  docker compose run --rm \
    -e ROS_DOMAIN_ID="${ros_domain}" \
    -e GZ_PARTITION="salus-runtime-keepout-${variant}-$$" \
    -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}-${variant}" \
    -e SMOKE_RUNTIME_DIR="/tmp/salus-smoke-runtime/${SMOKE_RUN_TOKEN:-direct}/keepout-${variant}" \
    ros2 bash -lc "
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      source /ros2_ws/install/setup.bash
      set -u
      source /ros2_ws/tools/smoke_harness.sh
      smoke_init runtime-keepout-${variant}
      trap smoke_cleanup EXIT

      free_world=\"\$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world\"
      no_keepout_params=\"\$(ros2 pkg prefix salus_navigation)/share/salus_navigation/config/nav2_core_no_obstacles_no_keepout_diag.yaml\"

      smoke_start_launch navigation \"ros2 launch salus_bringup integration_sim.launch.py world:=\${free_world} capability_profile:=no_obstacle_detection ${launch_args}\"
      smoke_start_runtime_timing_probe
      smoke_run navigation_contract \"EXPECT_NO_OBSTACLE_DETECTION=1 python3 /ros2_ws/tools/smoke_navigation_core_sim.py\"
      smoke_note \"runtime_keepout_isolation_variant:${variant}\"
    "
}

# B: same navigation contract, but no zones/map-server path and no KeepoutFilter.
run_variant "B-no-zones" "64" \
  "launch_zones:=false use_keepout:=false nav2_no_obstacles_params_file:=\${no_keepout_params}"

# C: current production-like zones/keepout composition with the same world and goal.
run_variant "C-current-keepout" "65" \
  "launch_zones:=true use_keepout:=true"
