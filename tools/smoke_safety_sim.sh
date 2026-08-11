#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=43
  export GZ_PARTITION="salus_safety_smoke_$$"
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
    if test "${exit_code}" -ne 0 && test -s "${integration_log}"; then tail -n 120 "${integration_log}"; fi
    return "${exit_code}"
  }
  trap cleanup EXIT
  for _attempt in $(seq 1 120); do
    if ros2 node list 2>/dev/null | grep -qx /nav_command_server && ros2 node list 2>/dev/null | grep -qx /collision_monitor; then break; fi
    sleep 0.25
  done
  ros2 node list | grep -qx /nav_command_server
  ros2 node list | grep -qx /collision_monitor
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  python3 /ros2_ws/tools/smoke_safety_sim.py
'
