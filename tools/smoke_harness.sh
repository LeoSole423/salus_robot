#!/usr/bin/env bash
# Shared, semantic smoke-test lifecycle helpers.  This file is sourced inside
# the ROS container; callers must source ROS before calling smoke_init.

smoke_init() {
  SMOKE_SCENARIO="$1"
  SMOKE_TIMEOUT_S="${SMOKE_TIMEOUT_S:-240}"
  SMOKE_STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  SMOKE_RUN_ID="${SMOKE_SCENARIO}-$(date -u +%Y%m%dT%H%M%S)-$$"
  SMOKE_ARTIFACT_DIR="${SMOKE_ARTIFACT_ROOT:-/ros2_ws/artifacts/smokes}/${SMOKE_RUN_ID}"
  SMOKE_LAUNCH_PIDS=()
  SMOKE_READY_EVENTS=()
  SMOKE_ARTIFACT_NAMES=()
  mkdir -p "${SMOKE_ARTIFACT_DIR}"
  export SMOKE_SCENARIO SMOKE_RUN_ID SMOKE_ARTIFACT_DIR
  printf '%s\n' "${ROS_DOMAIN_ID:-}" >"${SMOKE_ARTIFACT_DIR}/ros_domain_id.txt"
  printf '%s\n' "${GZ_PARTITION:-}" >"${SMOKE_ARTIFACT_DIR}/gz_partition.txt"
  printf '%s\n' "${SMOKE_RUNTIME_DIR:-}" >"${SMOKE_ARTIFACT_DIR}/runtime_dir.txt"
}

smoke_reserve_artifact_name() {
  local name="$1" existing
  for existing in "${SMOKE_ARTIFACT_NAMES[@]:-}"; do
    if [[ "${existing}" == "${name}" ]]; then
      printf 'Smoke artifact name %q is already reserved in scenario %s\n' \
        "${name}" "${SMOKE_SCENARIO}" >&2
      return 1
    fi
  done
  SMOKE_ARTIFACT_NAMES+=("${name}")
}

smoke_note() {
  SMOKE_READY_EVENTS+=("$1")
  printf '[smoke:%s] %s\n' "${SMOKE_SCENARIO}" "$1"
}

smoke_start_launch() {
  local name="$1" command="$2" log_file
  smoke_reserve_artifact_name "${name}"
  log_file="${SMOKE_ARTIFACT_DIR}/${name}.log"
  # Own a process group so cleanup also reaches ros2 launch descendants.
  setsid bash -lc "${command}" >"${log_file}" 2>&1 &
  SMOKE_LAUNCH_PIDS+=("$!")
  printf '%s\n' "${SMOKE_LAUNCH_PIDS[@]}" >"${SMOKE_ARTIFACT_DIR}/launch_pids.txt"
  smoke_note "launch_started:${name}"
}

# Run a smoke assertion while preserving its stdout/stderr beside the launch
# log.  This makes failures actionable without relying on CI's truncated
# terminal output.
smoke_run() {
  local name="$1" command="$2" log_file
  smoke_reserve_artifact_name "${name}"
  log_file="${SMOKE_ARTIFACT_DIR}/${name}.log"
  if ! bash -lc "${command}" >"${log_file}" 2>&1; then
    printf 'Smoke assertion %s failed; output follows:\n' "${name}" >&2
    cat "${log_file}" >&2 || true
    return 1
  fi
  # The full assertion output is an artifact.  Avoid streaming it back on a
  # successful run: some ROS CLI tools emit terminal-control or NUL bytes
  # which make CI logs unreadable.
  printf 'Smoke assertion %s passed; log: %s\n' "${name}" "${log_file}"
  smoke_note "passed:${name}"
}

smoke_wait() {
  local description="$1" timeout_s="$2" command="$3"
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if eval "${command}" >/dev/null 2>&1; then
      smoke_note "ready:${description}"
      return 0
    fi
    sleep 0.25
  done
  smoke_note "timeout:${description}"
  printf 'Smoke timeout: expected %s within %ss\n' "${description}" "${timeout_s}" >&2
  return 1
}

smoke_wait_node() {
  local node="$1" timeout_s="${2:-30}"
  smoke_wait "node:${node}" "${timeout_s}" "nodes=\"\$(timeout 2 ros2 node list 2>/dev/null || true)\"; grep -qx '${node}' <<<\"\${nodes}\""
}

smoke_wait_topic() {
  local topic="$1" timeout_s="${2:-30}"
  smoke_wait "topic:${topic}" "${timeout_s}" "topics=\"\$(timeout 2 ros2 topic list 2>/dev/null || true)\"; grep -qx '${topic}' <<<\"\${topics}\""
}

smoke_wait_topic_message() {
  local topic="$1" timeout_s="${2:-30}"
  smoke_wait "message:${topic}" "${timeout_s}" "timeout 2 ros2 topic echo '${topic}' --once >/dev/null 2>&1"
}

smoke_wait_lifecycle() {
  local node="$1" timeout_s="${2:-30}"
  local safe_node report_path
  safe_node="${node#/}"
  safe_node="${safe_node//\//_}"
  report_path="${SMOKE_ARTIFACT_DIR}/lifecycle_${safe_node}.json"
  if python3 /ros2_ws/tools/lifecycle_readiness_probe.py \
    --node "${node}" --timeout "${timeout_s}" --report-path "${report_path}"; then
    smoke_note "ready:lifecycle:${node}:active"
    return 0
  fi
  smoke_note "timeout:lifecycle:${node}:active"
  printf 'Smoke timeout: expected lifecycle %s active within %ss; report: %s\n' \
    "${node}" "${timeout_s}" "${report_path}" >&2
  return 1
}

smoke_collect_diagnostics() {
  local status="$1"
  # Collect while the launch is still alive.  Each query is bounded so a
  # broken discovery service cannot prevent process cleanup.
  timeout --kill-after=1s 2s ros2 node list >"${SMOKE_ARTIFACT_DIR}/nodes.txt" 2>&1 || true
  timeout --kill-after=1s 2s ros2 topic list -t >"${SMOKE_ARTIFACT_DIR}/topics.txt" 2>&1 || true
  timeout --kill-after=1s 2s ros2 service list -t >"${SMOKE_ARTIFACT_DIR}/services.txt" 2>&1 || true
  timeout --kill-after=1s 2s ros2 param list >"${SMOKE_ARTIFACT_DIR}/parameters.txt" 2>&1 || true
  for node in /collision_monitor /bt_navigator /planner_server /controller_server \
    /keepout_filter_mask_server /route_executor; do
    timeout --kill-after=1s 2s ros2 lifecycle get "${node}" >"${SMOKE_ARTIFACT_DIR}/lifecycle${node//\//_}.txt" 2>&1 || true
  done
  timeout --kill-after=1s 2s ros2 topic info /tf -v >"${SMOKE_ARTIFACT_DIR}/tf_publishers.txt" 2>&1 || true
  timeout --kill-after=1s 2s ros2 topic info /tf_static -v >"${SMOKE_ARTIFACT_DIR}/tf_static_publishers.txt" 2>&1 || true
  : >"${SMOKE_ARTIFACT_DIR}/qos.txt"
  for topic in /odometry/global /scan_clean /cmd_vel_final /path_health; do
    printf '\n## %s\n' "${topic}" >>"${SMOKE_ARTIFACT_DIR}/qos.txt"
    timeout --kill-after=1s 2s ros2 topic info "${topic}" -v >>"${SMOKE_ARTIFACT_DIR}/qos.txt" 2>&1 || true
  done
  : >"${SMOKE_ARTIFACT_DIR}/last_states.txt"
  for topic in /nav_command_server/telemetry /nav_command_server/events /path_health \
    /route_executor/state; do
    printf '\n## %s\n' "${topic}" >>"${SMOKE_ARTIFACT_DIR}/last_states.txt"
    timeout --kill-after=1s 2s ros2 topic echo "${topic}" --once >>"${SMOKE_ARTIFACT_DIR}/last_states.txt" 2>&1 || true
  done
  python3 - "${SMOKE_ARTIFACT_DIR}/report.json" "${SMOKE_SCENARIO}" \
    "${SMOKE_RUN_ID}" "${SMOKE_STARTED_AT}" "${status}" "${SMOKE_TIMEOUT_S}" \
    "${SMOKE_READY_EVENTS[*]}" <<'PY'
import json
import sys

path, scenario, run_id, started, status, timeout_s, events = sys.argv[1:]
import os
with open(path, "w", encoding="utf-8") as report:
    json.dump({
        "scenario": scenario,
        "run_id": run_id,
        "started_at": started,
        "status": status,
        "smoke_timeout_s": int(timeout_s),
        "readiness": events.split(),
        "isolation": {
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "gz_partition": os.environ.get("GZ_PARTITION", ""),
            "run_token": os.environ.get("SMOKE_RUN_TOKEN", ""),
            "runtime_dir": os.environ.get("SMOKE_RUNTIME_DIR", ""),
        },
    }, report, indent=2, sort_keys=True)
    report.write("\n")
PY
}

smoke_cleanup() {
  local status=$?
  trap - EXIT
  smoke_collect_diagnostics "${status}"
  for pid in "${SMOKE_LAUNCH_PIDS[@]:-}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
  done
  local cleanup_deadline=$((SECONDS + 5)) pid alive
  while (( SECONDS < cleanup_deadline )); do
    alive=0
    for pid in "${SMOKE_LAUNCH_PIDS[@]:-}"; do
      if kill -0 "${pid}" 2>/dev/null; then alive=1; break; fi
    done
    (( alive == 0 )) && break
    sleep 0.1
  done
  for pid in "${SMOKE_LAUNCH_PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill -KILL -- "-${pid}" 2>/dev/null || kill -KILL "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
  done
  if (( status != 0 )); then
    printf 'Smoke %s failed; diagnostics: %s\n' "${SMOKE_SCENARIO}" "${SMOKE_ARTIFACT_DIR}" >&2
    for log in "${SMOKE_ARTIFACT_DIR}"/*.log; do
      test -e "${log}" && tail -n 100 "${log}" || true
    done
  fi
  return "${status}"
}
