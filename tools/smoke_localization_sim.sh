#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm ros2 bash -lc '
  set -eo pipefail
  export ROS_DOMAIN_ID=42 GZ_PARTITION="salus_localization_$$"
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init localization-free-world
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch motion_launch "ros2 launch salus_simulation motion_sim.launch.py world:=${free_world}"
  smoke_start_launch control_launch "ros2 launch salus_control control_sim.launch.py use_sim_time:=true"
  smoke_start_launch localization_launch "ros2 launch salus_localization localization_sim.launch.py"
  smoke_wait_topic /clock 30
  smoke_wait_node /ackermann_odometry 40
  smoke_wait_node /ekf_filter_node_local 40
  smoke_wait_topic /wheel/odometry 30
  smoke_wait_topic /imu/data 30
  smoke_wait_topic /odometry/local 30
  test "$(ros2 topic type /wheel/odometry)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /imu/data)" = "sensor_msgs/msg/Imu"
  test "$(ros2 topic type /odometry/local)" = "nav_msgs/msg/Odometry"
  smoke_run localization_probe "python3 /ros2_ws/tools/smoke_localization_sim.py"
'
