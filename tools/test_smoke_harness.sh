#!/usr/bin/env bash
# Fast, ROS-independent regression checks for bounded semantic readiness.
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
temporary_root="$(mktemp -d)"
cleanup() { rm -rf "${temporary_root}"; }
trap cleanup EXIT

export SMOKE_ARTIFACT_ROOT="${temporary_root}"
source "${repo_dir}/tools/smoke_harness.sh"
smoke_init harness-selftest

started="${SECONDS}"
if smoke_wait "injected:missing-process" 1 false; then
  echo "missing process unexpectedly became ready" >&2
  exit 1
fi
elapsed=$((SECONDS - started))
if (( elapsed > 3 )); then
  echo "bounded readiness timeout exceeded its diagnostic budget" >&2
  exit 1
fi
[[ " ${SMOKE_READY_EVENTS[*]} " == *" timeout:injected:missing-process "* ]]

if ! smoke_wait "injected:available-service" 1 true; then
  echo "available condition was not recognised" >&2
  exit 1
fi
[[ " ${SMOKE_READY_EVENTS[*]} " == *" ready:injected:available-service "* ]]
echo "Smoke harness self-test passed"
