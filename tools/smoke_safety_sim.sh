#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-43}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-safety-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init safety-synthetic-scan
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch motion "ros2 launch salus_simulation motion_sim.launch.py world:=${free_world}"
  smoke_start_launch control "ros2 launch salus_control control_sim.launch.py use_sim_time:=true"
  smoke_start_launch odom_tf "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_footprint"
  smoke_start_launch arbitration "ros2 launch salus_navigation safety_arbitration_sim.launch.py use_sim_time:=true"
  smoke_wait_node /controller_server 40
  smoke_wait_node /nav_command_server 40
  smoke_wait_node /collision_monitor 40
  smoke_wait_lifecycle /collision_monitor 40
  smoke_wait_topic /clock 30
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  smoke_run safety_probe "python3 /ros2_ws/tools/smoke_safety_sim.py --report-path ${SMOKE_ARTIFACT_DIR}/safety_probe.json"
  smoke_note "safety_arbitration_valid"
'
