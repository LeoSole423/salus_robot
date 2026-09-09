#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
usage() {
  echo "Usage: ./tools/nav_eval.sh run <scenario.yaml> [output-dir]" >&2
  echo "       ./tools/nav_eval.sh observe [output-dir]" >&2
  echo "       ./tools/nav_eval.sh matrix-summary <matrix.yaml> <output-dir> <trial-dir>..." >&2
  echo "       ./tools/nav_eval.sh matrix <matrix.yaml> [output-dir]" >&2
  echo "       ./tools/nav_eval.sh isolation <output-dir> [options]" >&2
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
    shift 3 || true
    mkdir -p "${output}"
    output="$(cd "${output}" && pwd)"
    matrix_run_token="salus-nav-matrix-$(date -u +%Y%m%dT%H%M%S)-$$"
    eval_lock_root="${TMPDIR:-/tmp}/salus-nav-evaluation-domains"
    mkdir -p "${eval_lock_root}"
    exec docker compose run --rm \
      -e "SALUS_NAV_EVAL_RUN_TOKEN=${matrix_run_token}" \
      -e SALUS_NAV_EVAL_LOCK_ROOT=/salus-nav-evaluation-domains \
      -e "FASTDDS_BUILTIN_TRANSPORTS=UDPv4" \
      -v "${eval_lock_root}:/salus-nav-evaluation-domains" \
      -v "${output}:/evaluation-artifacts" ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /ros2_ws/install/setup.bash
      ros2 run salus_evaluation navigation_matrix_execute '${matrix}' /evaluation-artifacts $*
    "
    ;;
  isolation)
    output="${2:-${repo_dir}/artifacts/evaluations/isolation-$(date -u +%Y%m%dT%H%M%S)}"
    shift 2 || true
    mkdir -p "${output}"
    output="$(cd "${output}" && pwd)"
    isolation_run_token="salus-nav-isolation-$(date -u +%Y%m%dT%H%M%S)-$$"
    eval_lock_root="${TMPDIR:-/tmp}/salus-nav-evaluation-domains"
    mkdir -p "${eval_lock_root}"
    exec docker compose run --rm \
      -e "SALUS_NAV_EVAL_RUN_TOKEN=${isolation_run_token}" \
      -e SALUS_NAV_EVAL_LOCK_ROOT=/salus-nav-evaluation-domains \
      -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
      -v "${eval_lock_root}:/salus-nav-evaluation-domains" \
      -v "${output}:/isolation-artifacts" ros2 bash -lc "
      source /opt/ros/humble/setup.bash
      source /ros2_ws/install/setup.bash
      python3 /ros2_ws/tools/nav_eval_isolation.py /isolation-artifacts $*
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
