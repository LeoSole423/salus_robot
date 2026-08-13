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
  for attempt in $(seq 1 160); do
    nodes="$(ros2 node list 2>/dev/null || true)"
    grep -qx /route_executor <<<"${nodes}" && break
    sleep 0.25
  done
  nodes="$(ros2 node list)"
  grep -qx /route_executor <<<"${nodes}"
  for node in /bt_navigator /planner_server /controller_server; do
    for attempt in $(seq 1 160); do
      lifecycle="$(ros2 lifecycle get "${node}" 2>/dev/null || true)"
      grep -q active <<<"${lifecycle}" && break
      sleep 0.25
    done
    lifecycle="$(ros2 lifecycle get "${node}")"
    grep -q active <<<"${lifecycle}"
  done
  for attempt in $(seq 1 160); do
    services="$(ros2 service list 2>/dev/null || true)"
    actions="$(ros2 action list 2>/dev/null || true)"
    if grep -qx /fromLL <<<"${services}" && grep -qx /navigate_to_pose <<<"${actions}"; then break; fi
    sleep 0.25
  done
  services="$(ros2 service list)"; actions="$(ros2 action list)"
  grep -qx /fromLL <<<"${services}"
  grep -qx /navigate_to_pose <<<"${actions}"
  python3 /ros2_ws/tools/smoke_route_executor_sim.py
'
