#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

docker compose run --rm \
  -e ROS_DOMAIN_ID="${SMOKE_ROS_DOMAIN_ID:-49}" \
  -e GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-obstacle-drag-costmap-$$}" \
  -e IGN_PARTITION="${SMOKE_GZ_PARTITION:-salus-obstacle-drag-costmap-$$}" \
  -e SMOKE_RUN_TOKEN="${SMOKE_RUN_TOKEN:-costmap}" \
  -e SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-/tmp/salus-smoke-runtime/costmap}" \
  -e SMOKE_SENSOR_PROFILE="${SMOKE_SENSOR_PROFILE:-clean}" \
  -e SMOKE_SENSOR_SEED="${SMOKE_SENSOR_SEED:-6400}" \
  -e SMOKE_SOURCE_SHA="$(git rev-parse HEAD)" \
  ros2 bash -lc '
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    source /ros2_ws/install/setup.bash
    set -u
    source /ros2_ws/tools/smoke_harness.sh
    world=/ros2_ws/install/salus_simulation/share/salus_simulation/worlds/obstacle_drag.world
    geometry=/ros2_ws/install/salus_simulation/share/salus_simulation/config/obstacle_drag_geometry.yaml
    run_case() {
      local case_name="$1" repetitions="$2"
      export SMOKE_RUN_TOKEN="costmap-${case_name}"
      export SMOKE_RUNTIME_DIR="/tmp/salus-smoke-runtime/costmap-${case_name}"
      smoke_init "obstacle-drag-costmap-${case_name}"
      trap smoke_cleanup EXIT
      smoke_start_launch obstacle_drag_launch \
        "ros2 launch salus_bringup integration_sim.launch.py world:=${world} launch_navigation:=true use_keepout:=false rviz:=false"
      smoke_wait_topic /local_costmap/costmap_raw 60
      smoke_start_launch obstacle_drag_costmap_probe \
        "python3 /ros2_ws/tools/smoke_obstacle_drag_costmap.py --geometry ${geometry} --repetitions ${repetitions} --metrics-path ${SMOKE_ARTIFACT_DIR}/obstacle_drag_costmap_metrics.json --ros-args -p use_sim_time:=true"
      smoke_start_launch obstacle_drag_maneuver \
        "python3 /ros2_ws/tools/obstacle_drag_maneuver.py --repetitions ${repetitions} --pause-s 6.0 --ros-args -p use_sim_time:=true"
      smoke_wait "${case_name} costmap measurement" 120 \
        "test -s ${SMOKE_ARTIFACT_DIR}/obstacle_drag_costmap_metrics.json"
      smoke_note "obstacle_drag_costmap_${case_name}"
      CASE_ARTIFACT_DIR="${SMOKE_ARTIFACT_DIR}"
      smoke_cleanup
      trap - EXIT
    }

    run_case control 1
    control_artifact="${CASE_ARTIFACT_DIR}"
    run_case repeated 2
    repeated_artifact="${CASE_ARTIFACT_DIR}"
    comparison_path="${repeated_artifact}/obstacle_drag_costmap_comparison.json"
    python3 - "${control_artifact}/obstacle_drag_costmap_metrics.json" \
      "${repeated_artifact}/obstacle_drag_costmap_metrics.json" "${comparison_path}" <<"PY"
import json
import sys

control_path, repeated_path, output_path = sys.argv[1:]
with open(control_path, encoding="utf-8") as stream:
    control = json.load(stream)
with open(repeated_path, encoding="utf-8") as stream:
    repeated = json.load(stream)
comparison = {
    "schema_version": 1,
    "control": control,
    "repeated": repeated,
    "comparison": {
        "same_world_and_seed": (
            control["world"] == repeated["world"]
            and control["geometry_fixture"] == repeated["geometry_fixture"]
            and control["simulation_sensors"] == repeated["simulation_sensors"]
        ),
        "control_repetitions": control["maneuver"]["repetitions"],
        "repeated_repetitions": repeated["maneuver"]["repetitions"],
        "costmap_reset_in_repeated": repeated["maneuver"]["costmap_reset"],
        "metrics": {},
    },
}
for map_name in ("local_costmap", "global_costmap"):
    control_map = control[map_name]
    repeated_map = repeated[map_name]
    comparison["comparison"]["metrics"][map_name] = {
        "trail_width_delta_m": (
            repeated_map["trail_width_p95_m"] - control_map["trail_width_p95_m"]
            if repeated_map["trail_width_p95_m"] is not None
            and control_map["trail_width_p95_m"] is not None else None
        ),
        "ghost_persistence_delta_s": (
            repeated_map["ghost_persistence_s"] - control_map["ghost_persistence_s"]
            if repeated_map["ghost_persistence_s"] is not None
            and control_map["ghost_persistence_s"] is not None else None
        ),
    }
with open(output_path, "w", encoding="utf-8") as stream:
    json.dump(comparison, stream, indent=2)
    stream.write("\n")
print(f"Obstacle-drag costmap control/repeated comparison: {output_path}")
PY
  '
