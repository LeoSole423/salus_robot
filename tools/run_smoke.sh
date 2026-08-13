#!/usr/bin/env bash
# Enforce the CI budget per scenario.  Individual smoke scripts still own
# their semantic operation timeouts and their EXIT trap collects diagnostics.
set -euo pipefail

timeout_s="${SMOKE_HARD_TIMEOUT_S:-240}"
if (($# == 0)); then
  echo "usage: $0 <smoke-script> [args...]" >&2
  exit 64
fi

echo "[smoke-runner] hard timeout: ${timeout_s}s; command: $*"
timeout --foreground --signal=TERM --kill-after=10s "${timeout_s}s" "$@"
