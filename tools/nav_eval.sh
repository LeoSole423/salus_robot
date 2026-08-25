#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  echo "Usage: ./tools/nav_eval.sh run <scenario.yaml> [output-dir]" >&2
  echo "       ./tools/nav_eval.sh observe [output-dir]" >&2
  echo "Runs against an already started sim_operational.launch.py instance." >&2
}
mode="${1:-}"
case "${mode}" in
  run)
    scenario="${2:-}"; test -n "${scenario}" || { usage; exit 2; }
    output="${3:-${repo_dir}/artifacts/evaluations/run-$(date -u +%Y%m%dT%H%M%S)}"
    ;;
  observe)
    scenario=""; output="${2:-${repo_dir}/artifacts/evaluations/observe-$(date -u +%Y%m%dT%H%M%S)}"
    ;;
  *) usage; exit 2 ;;
esac
mkdir -p "${output}"
exec docker compose run --rm -v "${output}:/evaluation-artifacts" ros2 bash -lc "
  source /opt/ros/humble/setup.bash
  source /ros2_ws/install/setup.bash
  ros2 run salus_evaluation navigation_evaluation --ros-args \
    -p use_sim_time:=true -p mode:=${mode} -p scenario:=${scenario} \
    -p output_dir:=/evaluation-artifacts
"
