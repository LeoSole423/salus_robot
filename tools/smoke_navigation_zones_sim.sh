#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=45 GZ_PARTITION="salus_zones_smoke_$$"
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash
  integration_log="$(mktemp)"; ros2 launch salus_bringup integration_sim.launch.py zones_runtime_dir:=runtime/zones-smoke >"${integration_log}" 2>&1 & integration_pid=$!
  cleanup() { status=$?; trap - EXIT; kill -TERM "${integration_pid}" 2>/dev/null || true; wait "${integration_pid}" 2>/dev/null || true; if test "${status}" -ne 0; then tail -n 180 "${integration_log}"; fi; return "${status}"; }; trap cleanup EXIT
  for _attempt in $(seq 1 160); do nodes="$(ros2 node list 2>/dev/null || true)"; if grep -qx /zones_manager <<<"${nodes}" && grep -qx /keepout_filter_mask_server <<<"${nodes}"; then break; fi; sleep .25; done
  grep -qx /zones_manager <<<"$(ros2 node list)"; grep -qx /keepout_filter_mask_server <<<"$(ros2 node list)"
  for _attempt in $(seq 1 120); do state="$(ros2 lifecycle get /keepout_filter_mask_server 2>/dev/null || true)"; if grep -q active <<<"${state}"; then break; fi; sleep .25; done
  grep -q active <<<"$(ros2 lifecycle get /keepout_filter_mask_server)"
  test "$(ros2 topic type /keepout_filter_mask)" = "nav_msgs/msg/OccupancyGrid"
  test "$(ros2 topic type /costmap_filter_info)" = "nav2_msgs/msg/CostmapFilterInfo"
  python3 /ros2_ws/tools/smoke_navigation_zones_sim.py
'
