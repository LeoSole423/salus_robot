#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm \
  -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-61}" \
  -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-operational-$$}" \
  -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" \
  -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" \
  ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init sim-operational
  trap smoke_cleanup EXIT
  web_port=$((18700 + ${ROS_DOMAIN_ID}))
  free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
  smoke_start_launch operational "ros2 launch salus_bringup sim_operational.launch.py headless:=true world:=${free_world} web_ws_port:=${web_port} runtime_dir:=${SMOKE_RUNTIME_DIR}/profile"
  smoke_run operational_probe "python3 /ros2_ws/tools/integration_probe.py --operational --timeout 90 --report-path ${SMOKE_ARTIFACT_DIR}/operational_probe.json"
  smoke_note "sim_operational_composition_valid"
'
