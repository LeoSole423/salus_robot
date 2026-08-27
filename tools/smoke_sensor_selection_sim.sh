#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm \
  -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-42}" \
  -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-sensor-selection-$$}" \
  -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" \
  -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" \
  ros2 bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
    source /ros2_ws/tools/smoke_harness.sh
    smoke_init sensor-selection-external-heading
    trap smoke_cleanup EXIT
    free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
    smoke_start_launch sensor_selection_launch \
      "ros2 launch salus_bringup integration_sim.launch.py world:=${free_world} launch_navigation:=false launch_web:=false launch_camera:=false imu_source:=imu_primary orientation_source:=external_heading"
    smoke_wait_node /imu_selector 40
    smoke_wait_node /orientation_source_selector 40
    smoke_wait_node /sim_external_heading_from_odom 40
    smoke_wait_topic /hardware/imu_primary/data 40
    smoke_wait_topic /imu/data 40
    smoke_wait_topic /heading/external 40
    smoke_wait_topic /localization/orientation 40
    test "$(ros2 topic type /imu/data)" = "sensor_msgs/msg/Imu"
    test "$(ros2 topic type /localization/orientation)" = "sensor_msgs/msg/Imu"
    imu_info="$(ros2 node info /imu_selector)"
    orientation_info="$(ros2 node info /orientation_source_selector)"
    grep -q "/hardware/imu_primary/data" <<<"${imu_info}"
    ! grep -q "/hardware/imu_secondary/data" <<<"${imu_info}"
    grep -q "/heading/external" <<<"${orientation_info}"
    ! grep -q "/gps/course_heading" <<<"${orientation_info}"
    if ros2 node list | grep -qx /gps_course_heading; then
      echo "course-over-ground estimator must not run in external-heading profile" >&2
      exit 1
    fi
    python3 /ros2_ws/tools/probe_sensor_capabilities.py
    echo "Explicit IMU and external-heading selection smoke passed"
  '
