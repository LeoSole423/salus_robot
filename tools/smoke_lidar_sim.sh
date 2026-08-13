#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=48 GZ_PARTITION="salus_lidar_$$"
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init lidar-obstacle
  trap smoke_cleanup EXIT
  smoke_start_launch lidar "ros2 launch salus_bringup integration_sim.launch.py launch_navigation:=false"
  smoke_wait_node /cloud_normalizer 40
  smoke_wait_node /scan_ground_filter 40
  smoke_wait_topic_message /scan_3d_raw 30
  smoke_wait_topic_message /scan_3d 30
  smoke_wait_topic_message /obstacles_cloud 30
  smoke_wait_topic_message /scan_clean 30
  smoke_run lidar "python3 /ros2_ws/tools/smoke_lidar_sim.py"
  smoke_note "lidar_obstacle_chain_valid"
'
