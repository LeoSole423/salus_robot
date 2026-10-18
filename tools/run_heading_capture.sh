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
extra_mounts=()
if [[ -n "${HEADING_SENSOR_FILE:-}" ]]; then
  extra_mounts+=(-v "${HEADING_SENSOR_FILE}:/ros2_ws/src/salus_simulation/config/sensor_profiles/independent_nominal.yaml:ro")
fi
capture_network="${HEADING_NETWORK:-none}"
capture_localhost=1
[[ "$capture_network" != bridge ]] || capture_localhost=0
capture_status=0
docker run --rm --network "$capture_network" --cpus "${HEADING_CPUS:-4}" \
  "${extra_mounts[@]}" -e HEADING_RECIPE -e HEADING_SEED -e HEADING_GLOBAL_CONFIG -e FASTDDS_BUILTIN_TRANSPORTS="${HEADING_FASTDDS_TRANSPORTS:-DEFAULT}" -e ROS_LOCALHOST_ONLY="$capture_localhost" -e ROS_DOMAIN_ID="$capture_domain" \
  -e GZ_PARTITION="salus-${capture_name}-${capture_domain}" \
  -e IGN_PARTITION="salus-${capture_name}-${capture_domain}" \
  -e SMOKE_ARTIFACT_ROOT="/ros2_ws/${capture_dir}/smokes" -e SMOKE_RUN_TOKEN="$capture_name" -e SMOKE_RUNTIME_DIR="/tmp/${capture_name}" \
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
    smoke_start_launch integration "ros2 launch salus_bringup integration_sim.launch.py world:=${free_world} launch_navigation:=false rviz:=false orientation_source:=course_over_ground sim_sensor_profile:=${HEADING_PROFILE} sim_sensor_seed:=${HEADING_SEED:-6400} ${debug_args} ${HEADING_GLOBAL_CONFIG:+global_ekf_params_file:=${HEADING_GLOBAL_CONFIG}}"
    # A persistent direct observer avoids poisoning ros2cli daemon discovery
    # when short-lived CLI processes are killed during parallel startup.
    python3 tools/wait_heading_ready.py
    smoke_note ready:node:/gps_course_heading
    if [[ "$HEADING_SHADOWS" == true ]]; then
      if ros2 param get /map_gps_absolute_measurement map_odom_topic >/dev/null 2>&1; then
        echo "Historical shadows require the pre-compensation mapper (f062c15); refusing double compensation."
        exit 2
      fi
      [[ "$HEADING_EKF_DEBUG" == true ]] || { echo "shadows require debug configs"; exit 2; }
      smoke_start_launch gps_compensation "python3 tools/heading_gps_lever_diagnostic.py --ros-args -p use_sim_time:=true"
      for variant in no_gps_position compensated_gps delayed_gps_reference; do
        smoke_start_launch "shadow_${variant}" "ros2 run robot_localization ekf_node --ros-args -r __node:=ekf_global_${variant} -r odometry/filtered:=/diagnostic/global_${variant} --params-file /ros2_ws/${HEADING_CAPTURE_DIR}/${variant}-debug.yaml -p use_sim_time:=true"
        smoke_wait_node "/ekf_global_${variant}" 60
      done
    fi
    capture_status=0
    smoke_run heading_capture "python3 tools/capture_heading_baseline.py --output ${HEADING_CAPTURE_DIR} --scenario ${HEADING_SCENARIO}" || capture_status=$?
    python3 tools/dump_heading_parameters.py "$HEADING_CAPTURE_DIR"
    dpkg-query -W ros-humble-robot-localization > "$HEADING_CAPTURE_DIR/robot-localization-version.txt"
    smoke_note heading_baseline_captured
    # Natural completion preserves the status through login-shell logout hooks.
    test "$capture_status" -eq 0
  ' > "$capture_dir/run.log" 2>&1 || capture_status=$?
if [[ -n "${HEADING_RECIPE:-}" && -f "$capture_dir/sequence.json" ]]; then
  python3 tools/heading_turn_metrics.py "$capture_dir"
elif [[ -f "$capture_dir/sequence.json" ]]; then
  python3 tools/summarize_heading_capture.py "$capture_dir"
fi
python3 - "$capture_dir" "$capture_domain" "$capture_scenario" "$capture_profile" "$capture_ekf_debug" "$capture_shadows" <<'PYPROVENANCE'
import hashlib, json, pathlib, subprocess, sys
root = pathlib.Path('.')
files = [*root.glob('src/salus_localization/salus_localization/*heading*.py'),
         *root.glob('src/salus_localization/config/localization_*sim.yaml'),
         root/'tools/capture_heading_baseline.py', root/'tools/heading_turn_campaign.py', root/'tools/heading_turn_metrics.py', root/'tools/run_heading_capture.sh',
         root/'tools/prepare_heading_debug.py', root/'tools/wait_heading_ready.py', root/'tools/dump_heading_parameters.py', root/'tools/heading_gps_lever_diagnostic.py',
         root/'src/salus_localization/salus_localization/map_gps_absolute_measurement.py',
         root/'src/salus_localization/salus_localization/gnss_antenna_compensation.py',
         root/'src/salus_description/urdf/components/sensor_mounts.xacro']
provenance = dict(head=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
    image_id=subprocess.check_output(['docker','image','inspect','salus-robot:humble-dev','--format','{{.Id}}'],text=True).strip(),
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    domain_id=int(sys.argv[2]), scenario=sys.argv[3], profile=sys.argv[4], seed=int(__import__('os').environ.get('HEADING_SEED','6400')),
    network=__import__('os').environ.get('HEADING_NETWORK','none'), cpu_limit=int(__import__('os').environ.get('HEADING_CPUS','4')), ekf_debug=sys.argv[5], diagnostic_shadows=sys.argv[6], dirty_diff_sha256=hashlib.sha256(subprocess.check_output(['git','diff'])).hexdigest())
pathlib.Path(sys.argv[1], 'provenance.json').write_text(json.dumps(provenance,indent=2))
PYPROVENANCE

exit "$capture_status"
