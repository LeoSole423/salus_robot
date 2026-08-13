#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=43 GZ_PARTITION="salus_safety_$$"
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init safety-synthetic-scan
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch safety "ros2 launch salus_bringup integration_sim.launch.py launch_navigation:=false world:=${free_world}"
  smoke_wait_node /nav_command_server 40
  smoke_wait_node /collision_monitor 40
  smoke_wait_lifecycle /collision_monitor 40
  smoke_wait_topic /clock 30
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  smoke_run safety "python3 /ros2_ws/tools/smoke_safety_sim.py"
  smoke_note "safety_arbitration_valid"
'
