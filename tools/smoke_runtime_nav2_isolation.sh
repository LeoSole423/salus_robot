#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

run_variant() {
  local variant="$1"
  local ros_domain="$2"
  local launch_nav2="$3"

  docker compose run --rm \
    -e ROS_DOMAIN_ID="${ros_domain}" \
    -e GZ_PARTITION="salus-runtime-nav2-${variant}-$$" \
    -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}-${variant}" \
    -e SMOKE_RUNTIME_DIR="/tmp/salus-smoke-runtime/${SMOKE_RUN_TOKEN:-direct}/nav2-${variant}" \
    ros2 bash -lc "
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      source /ros2_ws/install/setup.bash
      set -u
      source /ros2_ws/tools/smoke_harness.sh
      smoke_init runtime-nav2-${variant}
      trap smoke_cleanup EXIT

      free_world=\"\$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world\"
      nav2_params=\"\$(ros2 pkg prefix salus_navigation)/share/salus_navigation/config/nav2_core_no_obstacles_no_keepout_diag.yaml\"

      smoke_start_launch motion \"ros2 launch salus_simulation motion_sim.launch.py world:=\${free_world}\"
      smoke_start_launch control \"ros2 launch salus_control control_sim.launch.py use_sim_time:=true\"
      smoke_start_launch localization \"ros2 launch salus_localization localization_sim.launch.py odometry_backend:=legacy\"
      smoke_start_launch global_localization \"ros2 launch salus_localization global_localization_sim.launch.py orientation_source:=course_over_ground\"
      smoke_start_launch capabilities \"ros2 launch salus_hardware capability_profile.launch.py profile:=no_obstacle_detection orientation_source:=course_over_ground\"

      if [[ \"${launch_nav2}\" == \"true\" ]]; then
        smoke_start_launch nav2 \"ros2 launch salus_navigation navigation_core_sim.launch.py use_keepout:=false obstacle_detection_required:=false nav2_params_file:=\${nav2_params}\"
      fi

      smoke_start_runtime_timing_probe

      smoke_wait_topic_message /odometry/local 45
      smoke_wait_topic_message /odometry/global 45
      if [[ \"${launch_nav2}\" == \"true\" ]]; then
        smoke_wait_lifecycle /bt_navigator 60
        smoke_wait_topic_message /local_costmap/costmap 30
      fi

      python3 - \"${SMOKE_ARTIFACT_DIR}/steady_window.json\" <<'PY'
import json
import sys
import time

path = sys.argv[1]
started = time.monotonic()
time.sleep(30.0)
completed = time.monotonic()
with open(path, "w", encoding="utf-8") as stream:
    json.dump({
        "started_monotonic_s": started,
        "completed_monotonic_s": completed,
        "duration_s": completed - started,
    }, stream, sort_keys=True)
    stream.write("\n")
PY
      smoke_note \"runtime_nav2_isolation_variant:${variant}\"
    "
}

# A: simulation + control + local/global localization only.
run_variant "A-localization" "70" "false"

# B: exactly A + Nav2, no obstacle layer and no keepout.
run_variant "B-nav2" "71" "true"
