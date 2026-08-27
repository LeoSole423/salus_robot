#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-41}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-control-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init control-battery
  trap smoke_cleanup EXIT
  smoke_start_launch control_launch "ros2 launch salus_control control_sim.launch.py command_input_mode:=canonical_vehicle_command"
  for topic in \
    /cmd_vel_final \
    /battery_state \
    /battery_mission_guard \
    /vehicle/command_shadow \
    /vehicle/command_shadow/diagnostics \
    /vehicle/command_dry_run/diagnostics \
    /cmd_vel_gazebo; do
    smoke_wait_topic "${topic}" 30
  done
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /battery_state)" = "sensor_msgs/msg/BatteryState"
  test "$(ros2 topic type /battery_mission_guard)" = "salus_interfaces/msg/BatteryMissionGuard"
  test "$(ros2 topic type /vehicle/command_shadow)" = "salus_interfaces/msg/VehicleCommand"
  test "$(ros2 topic type /vehicle/command_shadow/diagnostics)" = "diagnostic_msgs/msg/DiagnosticArray"
  test "$(ros2 topic type /vehicle/command_dry_run/diagnostics)" = "diagnostic_msgs/msg/DiagnosticArray"
  test "$(ros2 topic type /cmd_vel_gazebo)" = "geometry_msgs/msg/Twist"
  test "$(ros2 topic info /cmd_vel_gazebo | awk -F: '\''/Publisher count/ {gsub(/ /, "", $2); print $2}'\'')" = "1"
  smoke_run control_probe "python3 /ros2_ws/tools/smoke_control_sim.py"
  smoke_note "battery_presets_validated"
  echo "Control simulation smoke test passed"
'
