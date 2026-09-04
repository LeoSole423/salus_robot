#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
timeout_s="${SALUS_READINESS_TIMEOUT_S:-30}"
probe_path="${repo_dir}/tools/real_mvp_readiness_probe.py"

if [[ ! "${timeout_s}" =~ ^[1-9][0-9]*$ ]]; then
  echo "SALUS_READINESS_TIMEOUT_S must be a positive integer" >&2
  exit 2
fi

probe_timeout_s=$((timeout_s - 1))
if ((probe_timeout_s < 1)); then
  probe_timeout_s=1
fi

probe_command="$(printf 'exec python3 -c %q %q' "$(<"${probe_path}")" "${probe_timeout_s}")"
output="$({
  timeout "${timeout_s}" "${repo_dir}/tools/real_runtime_exec.sh" -- bash -lc "${probe_command}"
} 2>&1)" || {
  echo "real MVP readiness probe failed" >&2
  echo "${output}" >&2
  exit 1
}

grep -Fxq "REAL_MVP_READINESS_ACTIVE" <<<"${output}" || {
  echo "real MVP readiness is not ACTIVE" >&2
  echo "${output}" >&2
  exit 1
}

echo "REAL_MVP_READINESS_ACTIVE"
