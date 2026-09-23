#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"
docker compose run --rm \
  -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-47}" \
  -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-route-recovery-$$}" \
  -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" \
  -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/route-recovery}" \
  -e SMOKE_ROUTE_AUTO_YAWS=1 \
  -e SMOKE_ROUTE_SCENARIO="${SMOKE_RECOVERY_ROUTE_SCENARIO:-recovery_partial}" \
  -e SMOKE_ROUTE_LEG_SPACING_M="${SMOKE_RECOVERY_LEG_SPACING_M:-0.0}" \
  -e SMOKE_ROUTE_ACTION_INDEX="${SMOKE_RECOVERY_ACTION_INDEX:--1}" \
  -e SMOKE_RECOVERY_CANCEL_IN_WAIT="${SMOKE_RECOVERY_CANCEL_IN_WAIT:-0}" \
  -e SMOKE_RECOVERY_CREDIT_COUNT="${SMOKE_RECOVERY_CREDIT_COUNT:-1}" \
  ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init route-recovery-partial
  trap smoke_cleanup EXIT
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch route_recovery "ros2 launch salus_bringup integration_sim.launch.py launch_routes:=true route_execution_mode:=adaptive_dense world:=${free_world}"
  smoke_run recovery_probe "python3 /ros2_ws/tools/smoke_route_recovery_sim.py"
  smoke_note "route_recovery_partial_progress_valid"
'
