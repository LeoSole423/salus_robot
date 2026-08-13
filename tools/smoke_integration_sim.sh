#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

# Composition only: functional motion, LiDAR and navigation each have their
# own isolated smoke. Keeping them out of this world avoids fixture coupling.
docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=42 GZ_PARTITION="salus_integration_smoke_$$"
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  log_file="$(mktemp)"
  ros2 launch salus_bringup integration_sim.launch.py >"${log_file}" 2>&1 &
  pid=$!
  cleanup() {
    status=$?
    trap - EXIT
    kill -TERM "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
    if test "${status}" -ne 0; then tail -n 180 "${log_file}" || true; fi
    return "${status}"
  }
  trap cleanup EXIT
  for node in /robot_state_publisher /controller_server /planner_server /collision_monitor; do
    for attempt in $(seq 1 160); do
      nodes="$(ros2 node list 2>/dev/null || true)"
      grep -qx "${node}" <<<"${nodes}" && break
      sleep 0.25
    done
    nodes="$(ros2 node list)"; grep -qx "${node}" <<<"${nodes}"
  done
  for node in /bt_navigator /collision_monitor; do
    for attempt in $(seq 1 160); do
      lifecycle="$(ros2 lifecycle get "${node}" 2>/dev/null || true)"
      grep -q active <<<"${lifecycle}" && break
      sleep 0.25
    done
    lifecycle="$(ros2 lifecycle get "${node}")"; grep -q active <<<"${lifecycle}"
  done
  for topic in /odometry/global /scan_clean; do
    timeout 30 bash -c "until ros2 topic echo ${topic} --once >/dev/null 2>&1; do sleep 0.25; done"
  done
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /odometry/local)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /odometry/global)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /scan_3d_raw)" = "sensor_msgs/msg/PointCloud2"
  test "$(ros2 topic type /scan_clean)" = "sensor_msgs/msg/LaserScan"
  nodes="$(ros2 node list)"
  test "$(grep -cx /robot_state_publisher <<<"${nodes}")" = "1"
  echo "Integrated structural simulation smoke test passed"
'
