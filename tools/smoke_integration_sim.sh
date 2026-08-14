#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-42}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-integration-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init integration-structure
  trap smoke_cleanup EXIT
  smoke_start_launch integration "ros2 launch salus_bringup integration_sim.launch.py"
  smoke_wait_node /robot_state_publisher 40
  smoke_wait_node /controller_server 40
  smoke_wait_node /planner_server 40
  smoke_wait_node /collision_monitor 40
  smoke_wait_lifecycle /bt_navigator 40
  smoke_wait_lifecycle /collision_monitor 40
  smoke_run integration_probe "python3 /ros2_ws/tools/integration_probe.py --timeout 30 --report-path ${SMOKE_ARTIFACT_DIR}/integration_probe.json"
  test "$(ros2 topic type /cmd_vel_final)" = "salus_interfaces/msg/CmdVelFinal"
  test "$(ros2 topic type /odometry/local)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /odometry/global)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /scan_3d_raw)" = "sensor_msgs/msg/PointCloud2"
  test "$(ros2 topic type /scan_clean)" = "sensor_msgs/msg/LaserScan"
  nodes="$(ros2 node list)"
  test "$(grep -cx /robot_state_publisher <<<"${nodes}")" = "1"
  smoke_note "structural_contracts_valid"
  echo "Integrated structural simulation smoke test passed"
'
