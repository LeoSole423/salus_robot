#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

run_variant() {
  local variant="$1"
  local ros_domain="$2"
  local pressure_mode="$3"
  local launch_web="$4"
  local web_port=$((18800 + ros_domain))

  docker compose run --rm \
    -e ROS_DOMAIN_ID="${ros_domain}" \
    -e GZ_PARTITION="salus-runtime-services-${variant}-$$" \
    -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-direct}-${variant}" \
    -e SMOKE_RUNTIME_DIR="/tmp/salus-smoke-runtime/${SMOKE_RUN_TOKEN:-direct}/services-${variant}" \
    ros2 bash -lc "
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      source /ros2_ws/install/setup.bash
      set -u
      source /ros2_ws/tools/smoke_harness.sh
      smoke_init runtime-services-${variant}
      trap smoke_cleanup EXIT

      world=\"\$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world\"
      mkdir -p \"\${SMOKE_RUNTIME_DIR}/zones\" \"\${SMOKE_RUNTIME_DIR}/web\"

      python3 - \"\${SMOKE_RUNTIME_DIR}/zones/no_go_zones.geojson\" <<'PY'
import json
import math
import sys

path = sys.argv[1]
datum_lat = -31.4858037
datum_lon = -64.2410570

def ll(x, y):
    return [
        datum_lon + x / (111_320.0 * math.cos(math.radians(datum_lat))),
        datum_lat + y / 111_320.0,
    ]

points = [ll(x, y) for x, y in (
    (40.0, 40.0), (42.0, 40.0), (42.0, 42.0),
    (40.0, 42.0), (40.0, 40.0),
)]
document = {
    \"type\": \"FeatureCollection\",
    \"features\": [{
        \"type\": \"Feature\",
        \"properties\": {\"id\": \"phase2_far_keepout\", \"enabled\": True},
        \"geometry\": {\"type\": \"Polygon\", \"coordinates\": [points]},
    }],
}
with open(path, \"w\", encoding=\"utf-8\") as stream:
    json.dump(document, stream, separators=(\",\", \":\"))
    stream.write(\"\\n\")
PY

      smoke_start_launch navigation \"ros2 launch salus_bringup integration_sim.launch.py world:=\${world} capability_profile:=no_obstacle_detection zones_runtime_dir:=\${SMOKE_RUNTIME_DIR}/zones launch_web:=${launch_web} web_ws_port:=${web_port} web_waypoints_file:=\${SMOKE_RUNTIME_DIR}/web/waypoints.yaml\"

      if [[ \"${pressure_mode}\" == \"direct\" ]]; then
        smoke_start_launch snapshot \"ros2 launch salus_navigation navigation_snapshot_sim.launch.py\"
      fi

      smoke_start_runtime_timing_probe
      smoke_run navigation_contract \"EXPECT_NO_OBSTACLE_DETECTION=1 python3 /ros2_ws/tools/smoke_navigation_core_sim.py\"
      smoke_run pressure_probe \"python3 /ros2_ws/tools/runtime_snapshot_web_pressure_probe.py --mode ${pressure_mode} --requests 5 --interval-s 0.5 --web-port ${web_port}\"
      smoke_note \"runtime_services_isolation_variant:${variant}\"
    "
}

# C: Nav2 + real persisted keepout; no Snapshot server and no Web gateway.
run_variant "C-keepout" "66" "none" "false"

# D: same composition + Snapshot server; five direct Snapshot requests.
run_variant "D-snapshot" "67" "direct" "false"

# E: same composition + Snapshot server + Web gateway; five equivalent
# Snapshot requests enter through the WebSocket bridge.
run_variant "E-web" "68" "web" "true"
