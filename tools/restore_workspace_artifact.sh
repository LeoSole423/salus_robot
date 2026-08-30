#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

archive="${1:?workspace artifact archive required}"
expected_sha="${2:-${GITHUB_SHA:-}}"
if [[ -z "${expected_sha}" ]]; then
  echo "workspace restore requires an exact commit SHA" >&2
  exit 2
fi
if [[ ! -f "${archive}" ]]; then
  echo "workspace artifact not found: ${archive}" >&2
  exit 2
fi

staging="$(mktemp -d)"
trap 'rm -rf "${staging}"' EXIT
tar -C "${staging}" -xzf "${archive}"

actual_sha="$(python3 - "${staging}/workspace-manifest.json" <<'PY'
import json
import sys
from pathlib import Path
payload = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
if payload.get("schema_version") != 1:
    raise SystemExit("unsupported workspace artifact manifest")
print(payload.get("commit_sha", ""))
PY
)"
if [[ "${actual_sha}" != "${expected_sha}" ]]; then
  echo "workspace artifact SHA mismatch: expected ${expected_sha}, got ${actual_sha}" >&2
  exit 3
fi

rm -rf build install
cp -a "${staging}/workspace/build" "${staging}/workspace/install" .
echo "[workspace-artifact] restored sha=${actual_sha}"
