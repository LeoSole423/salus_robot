#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  echo "Usage: ./tools/nav_eval.sh run <scenario.yaml> [output-dir]" >&2
  echo "       ./tools/nav_eval.sh observe [output-dir]" >&2
  echo "       ./tools/nav_eval.sh matrix-summary <matrix.yaml> <output-dir> <trial-dir>..." >&2
  echo "       ./tools/nav_eval.sh matrix <matrix.yaml> [output-dir]" >&2
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
  matrix-summary)
    matrix="${2:-}"; output="${3:-}"
    test -n "${matrix}" && test -n "${output}" && test "$#" -ge 4 || { usage; exit 2; }
    mkdir -p "${output}"
    output="$(cd "${output}" && pwd)"
    exec docker compose run --rm -v "${output}:/matrix-artifacts" ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /ros2_ws/install/setup.bash
      ros2 run salus_evaluation navigation_matrix_summary '${matrix}' /matrix-artifacts ${*:4}
    "
    ;;
  matrix)
    matrix="${2:-}"; test -n "${matrix}" || { usage; exit 2; }
    output="${3:-${repo_dir}/artifacts/evaluations/matrix-$(date -u +%Y%m%dT%H%M%S)}"
    mkdir -p "${output}"
    output="$(cd "${output}" && pwd)"
    matrix_domain_id="$(( (RANDOM % 100) + 100 ))"
    matrix_gz_partition="salus-nav-matrix-${matrix_domain_id}-$$"
    exec docker compose run --rm -e "ROS_DOMAIN_ID=${matrix_domain_id}" \
      -e "GZ_PARTITION=${matrix_gz_partition}" \
      -v "${output}:/evaluation-artifacts" ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /ros2_ws/install/setup.bash
      ros2 run salus_evaluation navigation_matrix_execute '${matrix}' /evaluation-artifacts
    "
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
