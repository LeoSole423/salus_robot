#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  motion_log="$(mktemp)"; control_log="$(mktemp)"; localization_log="$(mktemp)"
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  ros2 launch salus_simulation motion_sim.launch.py world:="${free_world}" >"${motion_log}" 2>&1 & motion_pid=$!
  ros2 launch salus_control control_sim.launch.py use_sim_time:=true >"${control_log}" 2>&1 & control_pid=$!
  ros2 launch salus_localization localization_sim.launch.py >"${localization_log}" 2>&1 & localization_pid=$!
  cleanup() {
    kill -TERM "${localization_pid}" "${control_pid}" "${motion_pid}" 2>/dev/null || true
    wait "${localization_pid}" 2>/dev/null || true; wait "${control_pid}" 2>/dev/null || true; wait "${motion_pid}" 2>/dev/null || true
  }
  trap cleanup EXIT
  for _attempt in $(seq 1 80); do
    if ros2 topic list 2>/dev/null | grep -qx "/clock"; then break; fi
    sleep 0.25
  done
  # Avoid closing ros2 stdout early: head-like pipelines make ros2cli
  # raise BrokenPipeError on some GitHub runners.
  clock_topics="$(ros2 topic list)"
  grep -qx "/clock" <<<"${clock_topics}"
  python3 /ros2_ws/tools/smoke_localization_sim.py
  test "$(ros2 topic type /wheel/odometry)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /imu/data)" = "sensor_msgs/msg/Imu"
  test "$(ros2 topic type /odometry/local)" = "nav_msgs/msg/Odometry"
'
