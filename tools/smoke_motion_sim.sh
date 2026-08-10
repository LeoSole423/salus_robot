#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  motion_log="$(mktemp)"
  control_log="$(mktemp)"
  ros2 launch salus_simulation motion_sim.launch.py >"${motion_log}" 2>&1 &
  motion_pid=$!
  ros2 launch salus_control control_sim.launch.py >"${control_log}" 2>&1 &
  control_pid=$!
  cleanup() {
    kill -TERM "${control_pid}" "${motion_pid}" 2>/dev/null || true
    wait "${control_pid}" 2>/dev/null || true
    wait "${motion_pid}" 2>/dev/null || true
  }
  trap cleanup EXIT
  for _attempt in $(seq 1 80); do
    if ros2 topic list 2>/dev/null | grep -qx "/clock"; then
      break
    fi
    sleep 0.25
  done
  ros2 topic list | grep -qx "/clock"
  python3 /ros2_ws/tools/smoke_motion_sim.py
  test "$(ros2 topic type /odom_raw)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /joint_states)" = "sensor_msgs/msg/JointState"
  test "$(ros2 topic info /tf -v | grep -c "Publisher count: 1")" -eq 1
'
