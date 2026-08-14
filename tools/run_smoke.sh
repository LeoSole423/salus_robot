#!/usr/bin/env bash
# Enforce the CI budget per scenario.  Individual smoke scripts still own
# their semantic operation timeouts and their EXIT trap collects diagnostics.
set -euo pipefail

timeout_s="${SMOKE_HARD_TIMEOUT_S:-240}"
if (($# == 0)); then
  echo "usage: $0 <smoke-script> [args...]" >&2
  exit 64
fi

# Allocate an isolated DDS domain for the lifetime of this invocation. CI jobs
# run on separate hosts, while flock prevents concurrent local scenarios from
# sharing a domain. The descriptor remains open until this runner exits.
smoke_lock_root="${TMPDIR:-/tmp}/salus-smoke-domains"
mkdir -p "${smoke_lock_root}"
if [[ -z "${SMOKE_ROS_DOMAIN_ID:-}" ]]; then
  # Rotate the starting point as well as locking active domains. Fast DDS may
  # retain discovery leases briefly after a process exits, so immediate reuse
  # of domain 80 defeats isolation in sequential burn-ins.
  exec 8>"${smoke_lock_root}/allocator.lock"
  flock 8
  smoke_state_file="${smoke_lock_root}/next-domain"
  smoke_domain_start="$(cat "${smoke_state_file}" 2>/dev/null || echo 80)"
  if (( smoke_domain_start < 80 || smoke_domain_start > 199 )); then
    smoke_domain_start=80
  fi
  for smoke_offset in $(seq 0 119); do
    smoke_domain=$((80 + (smoke_domain_start - 80 + smoke_offset) % 120))
    eval "exec 9>${smoke_lock_root}/domain-${smoke_domain}.lock"
    if flock -n 9; then
      export SMOKE_ROS_DOMAIN_ID="${smoke_domain}"
      printf '%s\n' "$((smoke_domain == 199 ? 80 : smoke_domain + 1))" >"${smoke_state_file}"
      break
    fi
    exec 9>&-
  done
  flock -u 8
  exec 8>&-
fi
if [[ -z "${SMOKE_ROS_DOMAIN_ID:-}" ]]; then
  echo "[smoke-runner] no isolated ROS_DOMAIN_ID is available" >&2
  exit 75
fi

smoke_token="${SMOKE_RUN_TOKEN:-$(date -u +%Y%m%dT%H%M%S)-$$-${RANDOM}}"
export SMOKE_RUN_TOKEN="${smoke_token}"
export SMOKE_GZ_PARTITION="${SMOKE_GZ_PARTITION:-salus-smoke-${smoke_token}}"
export SMOKE_RUNTIME_DIR="${SMOKE_RUNTIME_DIR:-${TMPDIR:-/tmp}/salus-smoke-runtime/${smoke_token}}"
mkdir -p "${SMOKE_RUNTIME_DIR}"
cleanup_runner() {
  rm -rf -- "${SMOKE_RUNTIME_DIR}"
}
trap cleanup_runner EXIT INT TERM

echo "[smoke-runner] hard timeout: ${timeout_s}s; domain: ${SMOKE_ROS_DOMAIN_ID}; partition: ${SMOKE_GZ_PARTITION}; command: $*"
timeout --foreground --signal=TERM --kill-after=10s "${timeout_s}s" "$@"
