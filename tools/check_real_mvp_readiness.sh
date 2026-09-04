#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
timeout_s="${SALUS_READINESS_TIMEOUT_S:-30}"

output="$({
  timeout "${timeout_s}" "${repo_dir}/tools/real_runtime_exec.sh" -- bash -lc \
    'exec ros2 topic echo --no-daemon /navigation_startup/diagnostics --once'
} 2>&1)" || {
  echo "real MVP readiness probe failed" >&2
  echo "${output}" >&2
  exit 1
}

grep -Fq "message: 'ACTIVE: ALL_NAV2_NODES_ACTIVE'" <<<"${output}" || {
  echo "real MVP readiness is not ACTIVE" >&2
  echo "${output}" >&2
  exit 1
}

echo "REAL_MVP_READINESS_ACTIVE"
