#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
runtime_exec="${repo_dir}/tools/real_runtime_exec.sh"

SALUS_NTRIP_CONFIG_PATH="${SALUS_NTRIP_CONFIG_PATH:-${repo_dir}/src/salus_hardware/config/rtk_sources.local.yaml}"
SALUS_FCU_URL="${SALUS_FCU_URL:-/dev/ttyACM0:921600}"
SALUS_SERIAL_PORT="${SALUS_SERIAL_PORT:-/dev/ttyUSB0}"
SALUS_USE_KEEPOUT="${SALUS_USE_KEEPOUT:-true}"
SALUS_ZONES_RUNTIME_DIR="${SALUS_ZONES_RUNTIME_DIR:-runtime/zones}"

if [[ ! -r "${SALUS_NTRIP_CONFIG_PATH}" ]]; then
  echo "NTRIP config is not readable: ${SALUS_NTRIP_CONFIG_PATH}" >&2
  exit 1
fi

case "${SALUS_NTRIP_CONFIG_PATH}" in
  "${repo_dir}/"*)
    ntrip_config_in_repo="${SALUS_NTRIP_CONFIG_PATH#"${repo_dir}/"}"
    ntrip_config_container="/ros2_ws/${ntrip_config_in_repo}"
    ;;
  *)
    echo "NTRIP config must be inside the repository src mount: ${repo_dir}/src/..." >&2
    exit 1
    ;;
esac

exec "${runtime_exec}" \
  --device /dev/ttyACM0 \
  --device /dev/ttyUSB0 \
  --container-name salus-robot-real-runtime \
  -- \
  bash -lc "exec ros2 launch salus_bringup real_mvp.launch.py \
    fcu_url:=${SALUS_FCU_URL} \
    ntrip_config_path:=${ntrip_config_container} \
    serial_port:=${SALUS_SERIAL_PORT} \
    use_keepout:=${SALUS_USE_KEEPOUT} \
    zones_runtime_dir:=${SALUS_ZONES_RUNTIME_DIR}"
