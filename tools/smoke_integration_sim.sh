#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  integration_log="$(mktemp)"
  ros2 launch salus_bringup integration_sim.launch.py >"${integration_log}" 2>&1 &
  integration_pid=$!
  cleanup() {
    kill -TERM "${integration_pid}" 2>/dev/null || true
    wait "${integration_pid}" 2>/dev/null || true
    if ! test -s "${integration_log}"; then return; fi
    tail -n 120 "${integration_log}"
  }
  trap cleanup EXIT
  for _attempt in $(seq 1 120); do
    topics="$(ros2 topic list 2>/dev/null || true)"
    if grep -qx "/scan_clean" <<<"${topics}" && grep -qx "/odometry/global" <<<"${topics}"; then
      break
    fi
    sleep 0.25
  done
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /odometry/local)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /odometry/global)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /scan_3d_raw)" = "sensor_msgs/msg/PointCloud2"
  test "$(ros2 topic type /scan_clean)" = "sensor_msgs/msg/LaserScan"
  test "$(ros2 node list | grep -cx /robot_state_publisher)" = "1"
'
