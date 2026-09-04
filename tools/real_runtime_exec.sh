#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/real_runtime_common.sh"

devices=()
while (($#)); do
  case "$1" in
    --device)
      if (($# < 2)); then
        echo "--device requires a path" >&2
        exit 2
      fi
      devices+=(--device "$2")
      shift 2
      ;;
    --)
      shift
      break
      ;;
    *)
      echo "expected -- before runtime command" >&2
      exit 2
      ;;
  esac
done

if (($# == 0)); then
  echo "usage: real_runtime_exec.sh [--device PATH]... -- COMMAND [ARGS...]" >&2
  exit 2
fi

if [[ -n "$(git -C "${SALUS_REPO_DIR}" status --porcelain)" ]]; then
  echo "working tree is dirty; run ./tools/prepare_real_runtime.sh" >&2
  exit 1
fi

prepared_file="${SALUS_REAL_CACHE_DIR}/prepared.env"
if [[ ! -f "${prepared_file}" ]]; then
  echo "runtime is not prepared; run ./tools/prepare_real_runtime.sh" >&2
  exit 1
fi

unset SOURCE_SHA IMAGE_ID RECIPE_HASH DEPS_HASH WORKSPACE_KEY WORKSPACE_DIR CACHE_DIR DEPS_CACHE_DIR PREPARED_AT
source "${prepared_file}"

current_source_sha="$(git -C "${SALUS_REPO_DIR}" rev-parse HEAD)"
current_recipe_hash="$(salus_recipe_hash "$(id -u)" "$(id -g)")"
current_deps_hash="$(salus_deps_hash)"
current_image_id="$(salus_image_id "${SALUS_REAL_IMAGE}" 2>/dev/null || true)"

fail_stale() {
  echo "runtime preparation is stale ($1); run ./tools/prepare_real_runtime.sh" >&2
  exit 1
}

[[ "${current_source_sha}" == "${SOURCE_SHA:-}" ]] || fail_stale source
[[ "${current_recipe_hash}" == "${RECIPE_HASH:-}" ]] || fail_stale recipe
[[ "${current_deps_hash}" == "${DEPS_HASH:-}" ]] || fail_stale dependencies
[[ -n "${IMAGE_ID:-}" && "${current_image_id}" == "${IMAGE_ID}" ]] || fail_stale image
[[ -n "${WORKSPACE_DIR:-}" ]] || fail_stale workspace
[[ -d "${WORKSPACE_DIR}/build" && -d "${WORKSPACE_DIR}/install" && -d "${WORKSPACE_DIR}/log" ]] \
  || fail_stale workspace

workspace_state="${WORKSPACE_DIR}/state.env"
[[ -f "${workspace_state}" ]] || fail_stale workspace-state
unset STATE_SOURCE_SHA STATE_IMAGE_ID STATE_RECIPE_HASH STATE_DEPS_HASH STATE_PREPARED_AT
source "${workspace_state}"
[[ "${STATE_SOURCE_SHA:-}" == "${SOURCE_SHA}" ]] || fail_stale workspace-source
[[ "${STATE_IMAGE_ID:-}" == "${IMAGE_ID}" ]] || fail_stale workspace-image
[[ "${STATE_RECIPE_HASH:-}" == "${RECIPE_HASH}" ]] || fail_stale workspace-recipe
[[ "${STATE_DEPS_HASH:-}" == "${DEPS_HASH}" ]] || fail_stale workspace-dependencies

docker_args=(run --rm --network host)
if ((${#devices[@]})); then
  docker_args+=("${devices[@]}")
fi
docker_args+=(
  -v "${SALUS_REPO_DIR}/src:/ros2_ws/src:ro"
  -v "${DEPS_CACHE_DIR:-${SALUS_REAL_CACHE_DIR}/dependencies/${DEPS_HASH}/src}:/ros2_external/src:ro"
  -v "${WORKSPACE_DIR}/build:/ros2_ws/build"
  -v "${WORKSPACE_DIR}/install:/ros2_ws/install"
  -v "${WORKSPACE_DIR}/log:/ros2_ws/log"
)

docker "${docker_args[@]}" "${IMAGE_ID}" "$@"
