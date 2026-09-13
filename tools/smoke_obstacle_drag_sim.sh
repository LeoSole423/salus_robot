#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm \
  -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-49}" \
  -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-obstacle-drag-$$}" \
  -e IGN_PARTITION="${SMOKE_GZ_PARTITION:-salus-obstacle-drag-$$}" \
  -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" \
  -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" \
  -e SMOKE_SENSOR_PROFILE="${SMOKE_SENSOR_PROFILE:-clean}" \
  -e SMOKE_SENSOR_SEED="${SMOKE_SENSOR_SEED:-6400}" \
  -e SMOKE_SOURCE_SHA="$(git rev-parse HEAD)" \
  ros2 bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    set -u
    source /ros2_ws/tools/smoke_harness.sh
    smoke_init obstacle-drag
    trap smoke_cleanup EXIT
    world=/ros2_ws/install/salus_simulation/share/salus_simulation/worlds/obstacle_drag.world
    geometry=/ros2_ws/install/salus_simulation/share/salus_simulation/config/obstacle_drag_geometry.yaml
    smoke_start_launch obstacle_drag_launch \
      "ros2 launch salus_bringup integration_sim.launch.py world:=${world} launch_navigation:=false rviz:=false"
    smoke_wait_node /cloud_normalizer 40
    smoke_wait_node /scan_ground_filter 40
    smoke_start_launch obstacle_drag_maneuver \
      "python3 /ros2_ws/tools/obstacle_drag_maneuver.py --ros-args -p use_sim_time:=true"
    smoke_run obstacle_drag_probe \
      "python3 /ros2_ws/tools/smoke_obstacle_drag_sim.py --geometry ${geometry} --report-path ${SMOKE_ARTIFACT_DIR}/obstacle_drag_probe.json --metrics-path ${SMOKE_ARTIFACT_DIR}/obstacle_drag_metrics.json"
    smoke_note obstacle_drag_geometry_visible
  '
