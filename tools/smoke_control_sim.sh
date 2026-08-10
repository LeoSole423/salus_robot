#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  set -u
  log_file="$(mktemp)"
  ros2 launch salus_control control_sim.launch.py >"${log_file}" 2>&1 &
  launch_pid=$!
  cleanup() {
    kill -TERM "${launch_pid}" 2>/dev/null || true
    wait "${launch_pid}" 2>/dev/null || true
  }
  trap cleanup EXIT
  for _attempt in $(seq 1 40); do
    if ros2 service list 2>/dev/null | grep -qx "/sim_battery/set_preset"; then
      break
    fi
    sleep 0.25
  done
  ros2 service list | grep -qx "/sim_battery/set_preset"
  ros2 service list | grep -qx "/sim_battery/set_state"
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /battery_state)" = "sensor_msgs/msg/BatteryState"
  test "$(ros2 topic type /battery_mission_guard)" = \
    "salus_interfaces/msg/BatteryMissionGuard"
  for preset in full under_load watching return_home_rest return_home_load \
    stale suspect unavailable; do
    ros2 service call /sim_battery/set_preset \
      salus_interfaces/srv/SetSimBatteryPreset "{preset: ${preset}}" | \
      grep -Eq "applied_preset[:=].*${preset}"
  done
  echo "Control simulation smoke test passed"
'
