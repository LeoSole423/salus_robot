#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=42
  export GZ_PARTITION="salus_smoke_$$"
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
    if test "${exit_code}" -ne 0 && test -s "${integration_log}"; then
      tail -n 120 "${integration_log}"
    fi
    return "${exit_code}"
  }
  trap cleanup EXIT
  for _attempt in $(seq 1 120); do
    topics="$(ros2 topic list 2>/dev/null || true)"
    nodes="$(ros2 node list 2>/dev/null || true)"
    if grep -qx "/scan_clean" <<<"${topics}" \
      && grep -qx "/odometry/global" <<<"${topics}" \
      && grep -qx "/controller_server" <<<"${nodes}" \
      && grep -qx "/robot_state_publisher" <<<"${nodes}" \
      && grep -qx "/cloud_normalizer" <<<"${nodes}" \
      && grep -qx "/scan_ground_filter" <<<"${nodes}" \
      && grep -qx "/scan_noise_filter" <<<"${nodes}"; then
      break
    fi
    sleep 0.25
  done
  nodes="$(ros2 node list)"
  grep -qx "/controller_server" <<<"${nodes}"
  grep -qx "/robot_state_publisher" <<<"${nodes}"
  grep -qx "/cloud_normalizer" <<<"${nodes}"
  grep -qx "/scan_ground_filter" <<<"${nodes}"
  grep -qx "/scan_noise_filter" <<<"${nodes}"
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /odometry/local)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /odometry/global)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /scan_3d_raw)" = "sensor_msgs/msg/PointCloud2"
  test "$(ros2 topic type /scan_clean)" = "sensor_msgs/msg/LaserScan"
  test "$(ros2 topic type /cmd_vel_safe)" = "geometry_msgs/msg/Twist"
  test "$(grep -cx /robot_state_publisher <<<"${nodes}")" = "1"
  for _attempt in $(seq 1 120); do
    collision_state="$(ros2 lifecycle get /collision_monitor 2>/dev/null || true)"
    nav_state="$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
    if grep -q "active" <<<"${collision_state}" && grep -q "active" <<<"${nav_state}"; then break; fi
    sleep 0.25
  done
  grep -q "active" <<<"$(ros2 lifecycle get /collision_monitor)"
  grep -q "active" <<<"$(ros2 lifecycle get /bt_navigator)"
  python3 /ros2_ws/tools/smoke_lidar_sim.py
  # Safety arbitration has its own isolated smoke.  This integrated world
  # intentionally contains a static obstacle for LiDAR validation, so a
  # "clear path" safety assertion here would be invalid by construction.
  python3 /ros2_ws/tools/smoke_motion_sim.py
  python3 /ros2_ws/tools/smoke_navigation_core_sim.py
'
