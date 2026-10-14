#!/usr/bin/env bash
# Manual, isolated simulation experiment; no hardware/deployment.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_workspace="${1:?Provide workspace containing a compatible build/ and install/}"
capture_name="${2:-heading-capture}"
capture_domain="${3:-117}"
capture_dir="artifacts/${capture_name}"
cd "$repo_dir"
test ! -e "$capture_dir/samples.jsonl" || { echo 'Refusing to overwrite capture'; exit 1; }
mkdir -p "$capture_dir"
docker run --rm --network none --cpus 4 \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="$capture_domain" \
  -e GZ_PARTITION="salus-${capture_name}-${capture_domain}" \
  -e IGN_PARTITION="salus-${capture_name}-${capture_domain}" \
  -e SMOKE_RUN_TOKEN="$capture_name" -e SMOKE_RUNTIME_DIR="/tmp/${capture_name}" \
  -e HEADING_CAPTURE_DIR="$capture_dir" \
  -v "$repo_dir:/ros2_ws" -v "$runtime_workspace/build:/ros2_ws/build:ro" \
  -v "$runtime_workspace/install:/ros2_ws/install:ro" -w /ros2_ws \
  salus-robot:humble-dev bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    source tools/smoke_harness.sh
    smoke_init heading-baseline
    trap smoke_cleanup EXIT
    free_world="$(ros2 pkg prefix salus_simulation)/share/salus_simulation/worlds/free.world"
    smoke_start_launch integration "ros2 launch salus_bringup integration_sim.launch.py world:=${free_world} launch_navigation:=false rviz:=false orientation_source:=course_over_ground sim_sensor_profile:=clean sim_sensor_seed:=6400"
    smoke_wait_node /gps_course_heading 60
    ros2 param dump /gps_course_heading > "$HEADING_CAPTURE_DIR/gps-parameters.yaml"
    smoke_run heading_capture "python3 tools/capture_heading_baseline.py --output ${HEADING_CAPTURE_DIR}"
    smoke_note heading_baseline_captured
  ' > "$capture_dir/run.log" 2>&1
python3 tools/summarize_heading_capture.py "$capture_dir"
