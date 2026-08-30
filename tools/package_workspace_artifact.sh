#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${repo_dir}"

sha="${1:-${GITHUB_SHA:-}}"
destination="${2:-${RUNNER_TEMP:-${repo_dir}/artifacts}/salus-workspace-${sha}.tar.gz}"

if [[ -z "${sha}" ]]; then
  echo "workspace artifact requires an exact commit SHA" >&2
  exit 2
fi
for path in build install; do
  if [[ ! -d "${path}" ]]; then
    echo "workspace artifact missing ${path}/" >&2
    exit 2
  fi
done

mkdir -p "$(dirname "${destination}")"
staging="$(mktemp -d)"
trap 'rm -rf "${staging}"' EXIT
mkdir -p "${staging}/workspace"
cp -a build install "${staging}/workspace/"
python3 - "${staging}/workspace-manifest.json" "${sha}" <<'PY'
import json
import sys
from pathlib import Path

path, sha = sys.argv[1:]
Path(path).write_text(
    json.dumps({"schema_version": 1, "commit_sha": sha}, sort_keys=True) + "\n",
    encoding="utf-8",
)
PY

tar -C "${staging}" -czf "${destination}" workspace workspace-manifest.json
bytes="$(stat -c %s "${destination}")"
echo "[workspace-artifact] sha=${sha} bytes=${bytes} path=${destination}"
