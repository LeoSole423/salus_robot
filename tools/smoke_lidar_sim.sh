#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-48}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-lidar-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init lidar-obstacle
  trap smoke_cleanup EXIT
  smoke_start_launch lidar_launch "ros2 launch salus_bringup integration_sim.launch.py launch_navigation:=false"
  smoke_wait_node /cloud_normalizer 40
  smoke_wait_node /scan_ground_filter 40
  # The persistent Python probe below subscribes to all sensor topics with
  # their real best-effort QoS.  Do not create short-lived ros2cli readers
  # here: discovery under CI can miss one sensor sample despite a healthy
  # perception chain.
  smoke_run lidar_probe "python3 /ros2_ws/tools/smoke_lidar_sim.py"
  smoke_note "lidar_obstacle_chain_valid"
'
