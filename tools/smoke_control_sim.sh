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
  smoke_start_launch control "ros2 launch salus_control control_sim.launch.py"
  smoke_wait "service:/sim_battery/set_preset" 20 "services=\"\$(ros2 service list 2>/dev/null || true)\"; grep -qx /sim_battery/set_preset <<<\"\${services}\""
  smoke_wait "service:/sim_battery/set_state" 20 "services=\"\$(ros2 service list 2>/dev/null || true)\"; grep -qx /sim_battery/set_state <<<\"\${services}\""
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /battery_state)" = "sensor_msgs/msg/BatteryState"
  test "$(ros2 topic type /battery_mission_guard)" = "salus_interfaces/msg/BatteryMissionGuard"
  for preset in full under_load watching return_home_rest return_home_load stale suspect unavailable; do
    response="$(timeout 15s ros2 service call /sim_battery/set_preset salus_interfaces/srv/SetSimBatteryPreset "{preset: ${preset}}")"
    grep -Eq "applied_preset[:=].*${preset}" <<<"${response}"
  done
  smoke_note "battery_presets_validated"
  echo "Control simulation smoke test passed"
'
