#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm \
  -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-47}" \
  -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-routes-$$}" \
  -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" \
  -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" \
  -e SMOKE_ROUTE_SCENARIO="${SMOKE_ROUTE_SCENARIO:-open}" \
  -e SMOKE_ROUTE_ACTION_INDEX="${SMOKE_ROUTE_ACTION_INDEX:--1}" \
  -e SMOKE_ROUTE_LEG_SPACING_M="${SMOKE_ROUTE_LEG_SPACING_M:-2.0}" \
  -e SMOKE_ROUTE_CHUNK_SPAN_M="${SMOKE_ROUTE_CHUNK_SPAN_M:-4.5}" \
  -e SMOKE_ROUTE_CHUNK_MAX_WAYPOINTS="${SMOKE_ROUTE_CHUNK_MAX_WAYPOINTS:-3}" \
  -e SMOKE_ROUTE_AUTO_YAWS="${SMOKE_ROUTE_AUTO_YAWS:-0}" \
  -e SMOKE_ROUTE_EXECUTION_MODE="${SMOKE_ROUTE_EXECUTION_MODE:-single_checkpoint}" \
  -e FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-DEFAULT}" \
  ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init routes-free-world
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch routes "ros2 launch salus_bringup integration_sim.launch.py launch_routes:=true route_execution_mode:=${SMOKE_ROUTE_EXECUTION_MODE} world:=${free_world}"
  smoke_run route_executor "python3 /ros2_ws/tools/smoke_route_executor_sim.py"
  if [[ "${SMOKE_ROUTE_SCENARIO}" != "takeover" ]]; then
    smoke_run navigation_profiles "python3 /ros2_ws/tools/smoke_navigation_profiles.py"
  else
    smoke_note "skipped:navigation_profiles:mission_remains_paused_after_takeover"
  fi
  smoke_note "route_mission_progress_valid"
'
