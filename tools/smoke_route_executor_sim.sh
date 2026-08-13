#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=47 GZ_PARTITION="salus_routes_smoke_$$"
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  log_file="$(mktemp)"
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  ros2 launch salus_bringup integration_sim.launch.py launch_routes:=true world:="${free_world}" >"${log_file}" 2>&1 &
  pid=$!
  trap "kill -TERM ${pid} 2>/dev/null || true; wait ${pid} 2>/dev/null || true" EXIT
  for attempt in $(seq 1 160); do ros2 node list 2>/dev/null | grep -qx /route_executor && break; sleep 0.25; done
  ros2 node list | grep -qx /route_executor
  # The executor is only meaningful once Nav2 has produced fresh costmaps.
  # Waiting on the semantic inputs avoids treating process discovery as
  # operational readiness and avoids relaxing PathHealth freshness rules.
  for topic in /global_costmap/costmap /local_costmap/costmap; do
    for attempt in $(seq 1 40); do
      if timeout 1 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then break; fi
    done
    timeout 2 ros2 topic echo "${topic}" --once >/dev/null 2>&1
  done
  python3 /ros2_ws/tools/smoke_route_executor_sim.py
'
