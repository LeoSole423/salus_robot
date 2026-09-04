#!/usr/bin/env bash
set -euo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/real_runtime_common.sh"

force_image_rebuild=false
force_workspace_rebuild=false
adopt_validated_image=false

usage() {
  cat >&2 <<'EOF'
Usage: prepare_real_runtime.sh [--adopt-validated-image]
       [--force-image-rebuild] [--force-workspace-rebuild]
EOF
}

while (($#)); do
  case "$1" in
    --adopt-validated-image)
      adopt_validated_image=true
      ;;
    --force-image-rebuild)
      force_image_rebuild=true
      ;;
    --force-workspace-rebuild)
      force_workspace_rebuild=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      usage
      exit 2
      ;;
  esac
  shift
done

if [[ -n "$(git -C "${SALUS_REPO_DIR}" status --porcelain)" ]]; then
  echo "working tree is dirty; commit/stash before preparing physical runtime" >&2
  exit 1
fi

source_sha="$(git -C "${SALUS_REPO_DIR}" rev-parse HEAD)"
uid="$(id -u)"
gid="$(id -g)"
recipe_hash="$(salus_recipe_hash "${uid}" "${gid}")"
deps_hash="$(salus_deps_hash)"

mkdir -p \
  "${SALUS_REAL_CACHE_DIR}/images" \
  "${SALUS_REAL_CACHE_DIR}/dependencies" \
  "${SALUS_REAL_CACHE_DIR}/workspace"

image_env="${SALUS_REAL_CACHE_DIR}/images/${recipe_hash}.env"
image_action=reused
image_id=

build_image() {
  "${SALUS_REPO_DIR}/tools/build_real_image.sh"
  image_id="$(salus_image_id "${SALUS_REAL_IMAGE}")"
  image_arch="$(salus_image_arch "${SALUS_REAL_IMAGE}")"
  salus_atomic_write "${image_env}" <<EOF
RECIPE_HASH=${recipe_hash}
IMAGE_ID=${image_id}
ARCH=${image_arch}
PREPARED_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
}

current_image_id="$(salus_image_id "${SALUS_REAL_IMAGE}" 2>/dev/null || true)"
if [[ "${force_image_rebuild}" == true ]]; then
  image_action=rebuilt
  build_image
elif [[ "${adopt_validated_image}" == true ]]; then
  if [[ -z "${current_image_id}" ]]; then
    echo "cannot adopt missing image: ${SALUS_REAL_IMAGE}" >&2
    exit 1
  fi
  image_id="${current_image_id}"
  image_arch="$(salus_image_arch "${SALUS_REAL_IMAGE}")"
  image_action=adopted
  salus_atomic_write "${image_env}" <<EOF
RECIPE_HASH=${recipe_hash}
IMAGE_ID=${image_id}
ARCH=${image_arch}
PREPARED_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
elif [[ -f "${image_env}" ]]; then
  unset RECIPE_HASH IMAGE_ID ARCH PREPARED_AT
  source "${image_env}"
  if [[ "${RECIPE_HASH:-}" == "${recipe_hash}" ]] \
      && [[ -n "${IMAGE_ID:-}" ]] \
      && [[ "${current_image_id}" == "${IMAGE_ID}" ]]; then
    image_id="${IMAGE_ID}"
  else
    image_action=rebuilt
    build_image
  fi
else
  image_action=rebuilt
  build_image
fi

deps_action=reused
deps_dir="${SALUS_REAL_CACHE_DIR}/dependencies/${deps_hash}"
deps_tmp=
cleanup() {
  if [[ -n "${deps_tmp}" && -d "${deps_tmp}" ]]; then
    rm -rf -- "${deps_tmp}"
  fi
}
trap cleanup EXIT

if [[ ! -f "${deps_dir}/.salus_complete" ]] \
    || [[ ! -d "${deps_dir}/src/rslidar_sdk" ]] \
    || [[ ! -d "${deps_dir}/src/rslidar_msg" ]]; then
  deps_action=imported
  deps_tmp="$(mktemp -d "${SALUS_REAL_CACHE_DIR}/dependencies/.tmp.XXXXXX")"
  docker run --rm --network bridge \
    -v "${SALUS_REPO_DIR}/dependencies.repos:/input/dependencies.repos:ro" \
    -v "${deps_tmp}:/output" \
    "${image_id}" bash -lc '
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      set -u
      cd /output
      vcs import --input /input/dependencies.repos .
      git -C src/rslidar_sdk submodule update --init --recursive
      test "$(git -C src/rslidar_sdk rev-parse HEAD)" = \
        "7c4ea25fada93442c3d390aa4ef05e240999b851"
      test "$(git -C src/rslidar_sdk rev-parse HEAD:src/rs_driver)" = \
        "cd358851ab65bf57fc7e321837be2a425305b298"
      test "$(git -C src/rslidar_msg rev-parse HEAD)" = \
        "fe8a95cb242bd294cc3d5e3422f2093fb49a56ee"
      touch /output/.salus_complete
    '
  if [[ ! -f "${deps_tmp}/.salus_complete" ]] \
      || [[ ! -d "${deps_tmp}/src/rslidar_sdk" ]] \
      || [[ ! -d "${deps_tmp}/src/rslidar_msg" ]]; then
    echo "dependency import did not produce a complete cache" >&2
    exit 1
  fi
  mv -- "${deps_tmp}" "${deps_dir}"
  deps_tmp=
fi

workspace_key="$(salus_workspace_key "${image_id}" "${deps_hash}")"
workspace_dir="${SALUS_REAL_CACHE_DIR}/workspace/${workspace_key}"
build_dir="${workspace_dir}/build"
install_dir="${workspace_dir}/install"
log_dir="${workspace_dir}/log"
state_file="${workspace_dir}/state.env"
mkdir -p "${workspace_dir}"

workspace_action=clean_build
needs_build=true
previous_sha=
if [[ -f "${state_file}" ]]; then
  unset STATE_SOURCE_SHA STATE_IMAGE_ID STATE_RECIPE_HASH STATE_DEPS_HASH STATE_PREPARED_AT
  source "${state_file}"
  previous_sha="${STATE_SOURCE_SHA:-}"
  if [[ "${force_image_rebuild}" == false ]] \
      && [[ "${force_workspace_rebuild}" == false ]] \
      && [[ "${STATE_IMAGE_ID:-}" == "${image_id}" ]] \
      && [[ "${STATE_RECIPE_HASH:-}" == "${recipe_hash}" ]] \
      && [[ "${STATE_DEPS_HASH:-}" == "${deps_hash}" ]] \
      && [[ "${previous_sha}" == "${source_sha}" ]]; then
    workspace_action=reused
    needs_build=false
  elif [[ "${force_image_rebuild}" == false ]] \
      && [[ "${force_workspace_rebuild}" == false ]] \
      && [[ -n "${previous_sha}" ]] \
      && git -C "${SALUS_REPO_DIR}" merge-base --is-ancestor "${previous_sha}" "${source_sha}"; then
    workspace_action=incremental_build
  fi
fi

if [[ "${needs_build}" == true ]]; then
  if [[ "${workspace_action}" == clean_build ]]; then
    rm -rf -- "${build_dir}" "${install_dir}" "${log_dir}"
    rm -f -- "${state_file}"
  fi
  mkdir -p "${build_dir}" "${install_dir}" "${log_dir}"
  # rs_driver's CMake configure_file() writes its generated config/version
  # files into the pinned external source tree, so only that cache mount is RW.
  docker run --rm --network none \
    -v "${SALUS_REPO_DIR}/src:/ros2_ws/src:ro" \
    -v "${deps_dir}/src:/ros2_external/src" \
    -v "${build_dir}:/ros2_ws/build" \
    -v "${install_dir}:/ros2_ws/install" \
    -v "${log_dir}:/ros2_ws/log" \
    "${image_id}" bash -lc '
      set -eo pipefail
      source /opt/ros/humble/setup.bash
      set -u
      colcon build \
        --base-paths /ros2_ws/src /ros2_external/src \
        --symlink-install \
        --event-handlers console_direct+
    '
  salus_atomic_write "${state_file}" <<EOF
STATE_SOURCE_SHA=${source_sha}
STATE_IMAGE_ID=${image_id}
STATE_RECIPE_HASH=${recipe_hash}
STATE_DEPS_HASH=${deps_hash}
STATE_PREPARED_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF
fi

if [[ ! -d "${build_dir}" || ! -d "${install_dir}" || ! -d "${log_dir}" ]]; then
  echo "workspace cache is incomplete" >&2
  exit 1
fi

salus_atomic_write "${SALUS_REAL_CACHE_DIR}/prepared.env" <<EOF
SOURCE_SHA=${source_sha}
IMAGE_ID=${image_id}
RECIPE_HASH=${recipe_hash}
DEPS_HASH=${deps_hash}
WORKSPACE_KEY=${workspace_key}
WORKSPACE_DIR=${workspace_dir}
CACHE_DIR=${SALUS_REAL_CACHE_DIR}
DEPS_CACHE_DIR=${deps_dir}/src
PREPARED_AT=$(date -u +%Y-%m-%dT%H:%M:%SZ)
EOF

printf 'SOURCE_SHA=%s\n' "${source_sha}"
printf 'IMAGE_ID=%s\n' "${image_id}"
printf 'RECIPE_HASH=%s\n' "${recipe_hash}"
printf 'DEPS_HASH=%s\n' "${deps_hash}"
printf 'IMAGE_ACTION=%s\n' "${image_action}"
printf 'DEPS_ACTION=%s\n' "${deps_action}"
printf 'WORKSPACE_ACTION=%s\n' "${workspace_action}"
printf 'CACHE_DIR=%s\n' "${SALUS_REAL_CACHE_DIR}"
