#!/usr/bin/env bash
# Manual, isolated simulation experiment; no hardware/deployment.
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_workspace="${1:?Provide workspace containing a compatible build/ and install/}"
capture_name="${2:-heading-capture}"
capture_domain="${3:-117}"
capture_dir="artifacts/${capture_name}"
capture_scenario="${4:-right}"
capture_profile="${5:-clean}"
capture_ekf_debug="${6:-false}"
capture_shadows="${7:-false}"
cd "$repo_dir"
test ! -e "$capture_dir/samples.jsonl" || { echo 'Refusing to overwrite capture'; exit 1; }
mkdir -p "$capture_dir"
docker run --rm --network none --cpus 4 \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="$capture_domain" \
  -e GZ_PARTITION="salus-${capture_name}-${capture_domain}" \
  -e IGN_PARTITION="salus-${capture_name}-${capture_domain}" \
  -e SMOKE_RUN_TOKEN="$capture_name" -e SMOKE_RUNTIME_DIR="/tmp/${capture_name}" \
  -e HEADING_SHADOWS="$capture_shadows" -e HEADING_EKF_DEBUG="$capture_ekf_debug" -e HEADING_CAPTURE_DIR="$capture_dir" -e HEADING_SCENARIO="$capture_scenario" -e HEADING_PROFILE="$capture_profile" \
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
    debug_args=""
    if [[ "$HEADING_EKF_DEBUG" == true ]]; then
      python3 tools/prepare_heading_debug.py "$HEADING_CAPTURE_DIR"
      debug_args="local_ekf_params_file:=/ros2_ws/${HEADING_CAPTURE_DIR}/local-debug.yaml global_ekf_params_file:=/ros2_ws/${HEADING_CAPTURE_DIR}/global-debug.yaml"
    fi
    smoke_start_launch integration "ros2 launch salus_bringup integration_sim.launch.py world:=${free_world} launch_navigation:=false rviz:=false orientation_source:=course_over_ground sim_sensor_profile:=${HEADING_PROFILE} sim_sensor_seed:=6400 ${debug_args}"
    smoke_wait_node /gps_course_heading 60
    if [[ "$HEADING_SHADOWS" == true ]]; then
      [[ "$HEADING_EKF_DEBUG" == true ]] || { echo "shadows require debug configs"; exit 2; }
      smoke_start_launch gps_compensation "python3 tools/heading_gps_lever_diagnostic.py --ros-args -p use_sim_time:=true"
      for variant in no_gps_position compensated_gps delayed_gps_reference; do
        smoke_start_launch "shadow_${variant}" "ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_global_${variant} -r odometry/filtered:=/diagnostic/global_${variant} --params-file /ros2_ws/${HEADING_CAPTURE_DIR}/${variant}-debug.yaml -p use_sim_time:=true"
        smoke_wait_node "/ekf_global_${variant}" 60
      done
    fi
    ros2 param dump /gps_course_heading > "$HEADING_CAPTURE_DIR/gps-parameters.yaml"
    smoke_run heading_capture "python3 tools/capture_heading_baseline.py --output ${HEADING_CAPTURE_DIR} --scenario ${HEADING_SCENARIO}"
    ros2 param dump /ekf_filter_node_local > "$HEADING_CAPTURE_DIR/local-ekf-parameters.yaml"
    ros2 param dump /ekf_filter_node_global > "$HEADING_CAPTURE_DIR/global-ekf-parameters.yaml"
    dpkg-query -W ros-humble-robot-localization > "$HEADING_CAPTURE_DIR/robot-localization-version.txt"
    smoke_note heading_baseline_captured
  ' > "$capture_dir/run.log" 2>&1
python3 tools/summarize_heading_capture.py "$capture_dir"
python3 - "$capture_dir" "$capture_domain" "$capture_scenario" "$capture_profile" "$capture_ekf_debug" "$capture_shadows" <<'PYPROVENANCE'
import hashlib, json, pathlib, subprocess, sys
root = pathlib.Path('.')
files = [*root.glob('src/salus_localization/salus_localization/*heading*.py'),
         *root.glob('src/salus_localization/config/localization_*sim.yaml'),
         root/'tools/capture_heading_baseline.py', root/'tools/run_heading_capture.sh',
         root/'tools/prepare_heading_debug.py', root/'tools/heading_gps_lever_diagnostic.py']
provenance = dict(head=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
    image_id=subprocess.check_output(['docker','image','inspect','salus-robot:humble-dev','--format','{{.Id}}'],text=True).strip(),
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    domain_id=int(sys.argv[2]), scenario=sys.argv[3], profile=sys.argv[4], seed=6400,
    network='none', cpu_limit=4, ekf_debug=sys.argv[5], diagnostic_shadows=sys.argv[6], dirty_diff_sha256=hashlib.sha256(subprocess.check_output(['git','diff'])).hexdigest())
pathlib.Path(sys.argv[1], 'provenance.json').write_text(json.dumps(provenance,indent=2))
PYPROVENANCE
