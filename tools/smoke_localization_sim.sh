#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-42}" -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-localization-$$}" -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" -e VEHICLE_IO_PROFILE="${VEHICLE_IO_PROFILE:-legacy}" ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init localization-free-world
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch motion_launch "ros2 launch salus_simulation motion_sim.launch.py world:=${free_world}"
  smoke_start_launch control_launch "ros2 launch salus_control control_sim.launch.py use_sim_time:=true"
  if [[ "${VEHICLE_IO_PROFILE}" == "canonical" ]]; then
    smoke_start_launch vehicle_io_launch "ros2 launch salus_bringup vehicle_io_sim.launch.py use_sim_time:=true"
    expected_odometry_node=/kinematic_ackermann_odometry
  elif [[ "${VEHICLE_IO_PROFILE}" == "legacy" ]]; then
    expected_odometry_node=/ackermann_odometry
  else
    echo "VEHICLE_IO_PROFILE must be legacy or canonical" >&2
    exit 2
  fi
  smoke_start_launch localization_launch "ros2 launch salus_localization localization_sim.launch.py odometry_backend:=${VEHICLE_IO_PROFILE}"
  smoke_wait_topic /clock 30
  smoke_wait_node "${expected_odometry_node}" 40
  smoke_wait_node /ekf_filter_node_local 40
  smoke_wait_topic /wheel/odometry 30
  smoke_wait_topic /imu/data 30
  smoke_wait_topic /odometry/local 30
  test "$(ros2 topic type /wheel/odometry)" = "nav_msgs/msg/Odometry"
  test "$(ros2 topic type /imu/data)" = "sensor_msgs/msg/Imu"
  test "$(ros2 topic type /odometry/local)" = "nav_msgs/msg/Odometry"
  smoke_run localization_probe "python3 /ros2_ws/tools/smoke_localization_sim.py"
'
