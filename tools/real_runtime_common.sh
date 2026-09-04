#!/usr/bin/env bash
set -euo pipefail

SALUS_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SALUS_REPO_DIR="$(cd "${SALUS_TOOLS_DIR}/.." && pwd)"
SALUS_REAL_IMAGE="${SALUS_REAL_IMAGE:-salus-robot:humble-real}"
SALUS_REAL_CACHE_DIR="${SALUS_REAL_CACHE_DIR:-${XDG_CACHE_HOME:-${HOME}/.cache}/salus_robot/real-runtime}"

salus_recipe_hash() {
  local uid="$1"
  local gid="$2"

  {
    printf 'Dockerfile.real\0'
    cat "${SALUS_REPO_DIR}/Dockerfile.real"
    printf 'entrypoint.sh\0'
    cat "${SALUS_REPO_DIR}/entrypoint.sh"
    printf 'USER_UID=%s\0' "${uid}"
    printf 'USER_GID=%s\0' "${gid}"
  } | sha256sum | awk '{print $1}'
}

salus_deps_hash() {
  sha256sum "${SALUS_REPO_DIR}/dependencies.repos" | awk '{print $1}'
}

salus_image_id() {
  docker image inspect "$1" --format '{{.Id}}'
}

salus_image_arch() {
  docker image inspect "$1" --format '{{.Architecture}}'
}

salus_atomic_write() {
  local destination="$1"
  local temporary

  temporary="$(mktemp "${destination}.tmp.XXXXXX")"
  cat >"${temporary}"
  mv -f -- "${temporary}" "${destination}"
}

salus_workspace_key() {
  local image_id="$1"
  local deps_hash="$2"

  image_id="${image_id//[^[:alnum:]_.-]/_}"
  printf '%s-%s\n' "${image_id}" "${deps_hash}"
}
