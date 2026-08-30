#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm \
  -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-62}" \
  -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-operational-persistence-$$}" \
  -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}" \
  -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/direct}" \
  ros2 bash -lc '
  set -eo pipefail
  source /opt/ros/humble/setup.bash; source /ros2_ws/install/setup.bash; set -u
  source /ros2_ws/tools/smoke_harness.sh
  smoke_init operational-persistence
  trap smoke_cleanup EXIT
  web_port=$((18700 + ${ROS_DOMAIN_ID}))
  profile_runtime="${SMOKE_RUNTIME_DIR}/profile"
  export SALUS_WEB_SMOKE_PORT="${web_port}"

  cockpit_port_open() {
    python3 - "${web_port}" <<PY
import socket
import sys

port = int(sys.argv[1])
with socket.socket() as sock:
    sock.settimeout(0.2)
    raise SystemExit(0 if sock.connect_ex(("127.0.0.1", port)) == 0 else 1)
PY
  }

  smoke_start_launch initial "ros2 launch salus_bringup persistence_contract.launch.py web_ws_port:=${web_port} runtime_dir:=${profile_runtime}"
  smoke_run seed_persistence "python3 /ros2_ws/tools/smoke_operational_persistence.py --mode seed"

  initial_pid="${SMOKE_LAUNCH_PIDS[0]}"
  kill -TERM -- "-${initial_pid}" 2>/dev/null || kill -TERM "${initial_pid}" 2>/dev/null || true
  wait "${initial_pid}" 2>/dev/null || true
  smoke_wait "initial cockpit endpoint stopped" 10 "! cockpit_port_open"
  smoke_note "initial_persistence_owners_stopped"

  smoke_start_launch restarted "ros2 launch salus_bringup persistence_contract.launch.py web_ws_port:=${web_port} runtime_dir:=${profile_runtime}"
  smoke_wait "restarted cockpit endpoint ready" 15 "cockpit_port_open"
  smoke_run verify_persistence "python3 /ros2_ws/tools/smoke_operational_persistence.py --mode verify"
  smoke_note "operational_persistence_valid"
'
