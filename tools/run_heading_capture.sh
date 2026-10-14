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
cd "$repo_dir"
test ! -e "$capture_dir/samples.jsonl" || { echo 'Refusing to overwrite capture'; exit 1; }
mkdir -p "$capture_dir"
docker run --rm --network none --cpus 4 \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID="$capture_domain" \
  -e GZ_PARTITION="salus-${capture_name}-${capture_domain}" \
  -e IGN_PARTITION="salus-${capture_name}-${capture_domain}" \
  -e SMOKE_RUN_TOKEN="$capture_name" -e SMOKE_RUNTIME_DIR="/tmp/${capture_name}" \
  -e HEADING_CAPTURE_DIR="$capture_dir" -e HEADING_SCENARIO="$capture_scenario" -e HEADING_PROFILE="$capture_profile" \
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
    smoke_start_launch integration "ros2 launch salus_bringup integration_sim.launch.py world:=${free_world} launch_navigation:=false rviz:=false orientation_source:=course_over_ground sim_sensor_profile:=${HEADING_PROFILE} sim_sensor_seed:=6400"
    smoke_wait_node /gps_course_heading 60
    ros2 param dump /gps_course_heading > "$HEADING_CAPTURE_DIR/gps-parameters.yaml"
    smoke_run heading_capture "python3 tools/capture_heading_baseline.py --output ${HEADING_CAPTURE_DIR} --scenario ${HEADING_SCENARIO}"
    smoke_note heading_baseline_captured
  ' > "$capture_dir/run.log" 2>&1
python3 tools/summarize_heading_capture.py "$capture_dir"
python3 - "$capture_dir" "$capture_domain" "$capture_scenario" "$capture_profile" <<'PYPROVENANCE'
import hashlib, json, pathlib, subprocess, sys
root = pathlib.Path('.')
files = [*root.glob('src/salus_localization/salus_localization/*heading*.py'),
         *root.glob('src/salus_localization/config/localization_*sim.yaml'),
         root/'tools/capture_heading_baseline.py', root/'tools/run_heading_capture.sh']
provenance = dict(head=subprocess.check_output(['git','rev-parse','HEAD'], text=True).strip(),
    image_id=subprocess.check_output(['docker','image','inspect','salus-robot:humble-dev','--format','{{.Id}}'],text=True).strip(),
    source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
    domain_id=int(sys.argv[2]), scenario=sys.argv[3], profile=sys.argv[4], seed=6400,
    network='none', cpu_limit=4, dirty_diff_sha256=hashlib.sha256(subprocess.check_output(['git','diff'])).hexdigest())
pathlib.Path(sys.argv[1], 'provenance.json').write_text(json.dumps(provenance,indent=2))
PYPROVENANCE
