#!/usr/bin/env bash
set -euo pipefail

failed=0
for variable in "$@"; do
  result="${!variable:-missing}"
  printf '%-16s %s\n' "${variable}" "${result}"
  if [[ "${result}" != "success" ]]; then
    failed=1
  fi
done
exit "${failed}"
