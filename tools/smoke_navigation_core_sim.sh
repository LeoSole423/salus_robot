#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=44
  export GZ_PARTITION="salus_navigation_smoke_$$"
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  integration_log="$(mktemp)"
  ros2 launch salus_bringup integration_sim.launch.py >"${integration_log}" 2>&1 &
  integration_pid=$!
  cleanup() {
    exit_code=$?
    trap - EXIT
    kill -TERM "${integration_pid}" 2>/dev/null || true
    wait "${integration_pid}" 2>/dev/null || true
    if test "${exit_code}" -ne 0 && test -s "${integration_log}"; then tail -n 180 "${integration_log}"; fi
    return "${exit_code}"
  }
  trap cleanup EXIT
  for _attempt in $(seq 1 160); do
    nodes="$(ros2 node list 2>/dev/null || true)"
    if grep -qx /planner_server <<<"${nodes}" && grep -qx /controller_server <<<"${nodes}" && grep -qx /bt_navigator <<<"${nodes}"; then break; fi
    sleep 0.25
  done
  nodes="$(ros2 node list)"
  grep -qx /planner_server <<<"${nodes}"
  grep -qx /controller_server <<<"${nodes}"
  grep -qx /smoother_server <<<"${nodes}"
  grep -qx /bt_navigator <<<"${nodes}"
  grep -qx /behavior_server <<<"${nodes}"
  for _attempt in $(seq 1 120); do
    lifecycle="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
    if grep -q "active" <<<"${lifecycle}"; then break; fi
    sleep 0.25
  done
  grep -q "active" <<<"$(ros2 lifecycle get /bt_navigator)"
  test "$(ros2 topic type /cmd_vel)" = "geometry_msgs/msg/Twist"
  test "$(ros2 topic type /cmd_vel_safe)" = "geometry_msgs/msg/Twist"
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /plan)" = "nav_msgs/msg/Path"
  test "$(ros2 topic type /global_costmap/costmap)" = "nav_msgs/msg/OccupancyGrid"
  test "$(ros2 topic type /local_costmap/costmap)" = "nav_msgs/msg/OccupancyGrid"
  python3 /ros2_ws/tools/smoke_navigation_core_sim.py
'
