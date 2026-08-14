#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=41
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init control-battery
  trap smoke_cleanup EXIT
  smoke_start_launch control_launch "ros2 launch salus_control control_sim.launch.py"
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /battery_state)" = "sensor_msgs/msg/BatteryState"
  test "$(ros2 topic type /battery_mission_guard)" = "salus_interfaces/msg/BatteryMissionGuard"
  smoke_run control_probe "python3 /ros2_ws/tools/smoke_control_sim.py"
  smoke_note "battery_presets_validated"
  echo "Control simulation smoke test passed"
'
