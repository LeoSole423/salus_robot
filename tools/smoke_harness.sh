#!/usr/bin/env bash
# Shared, semantic smoke-test lifecycle helpers.  This file is sourced inside
# the ROS container; callers must source ROS before calling smoke_init.

smoke_init() {
  SMOKE_SCENARIO="$1"
  SMOKE_TIMEOUT_S="${SMOKE_TIMEOUT_S:-240}"
  SMOKE_STARTED_AT="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
  SMOKE_RUN_ID="${SMOKE_SCENARIO}-$(date -u +%Y%m%dT%H%M%S)-$$"
  SMOKE_ARTIFACT_DIR="/ros2_ws/artifacts/smokes/${SMOKE_RUN_ID}"
  SMOKE_LAUNCH_PIDS=()
  SMOKE_READY_EVENTS=()
  mkdir -p "${SMOKE_ARTIFACT_DIR}"
  export SMOKE_SCENARIO SMOKE_RUN_ID SMOKE_ARTIFACT_DIR
}

smoke_note() {
  SMOKE_READY_EVENTS+=("$1")
  printf '[smoke:%s] %s\n' "${SMOKE_SCENARIO}" "$1"
}

smoke_start_launch() {
  local name="$1" command="$2" log_file
  log_file="${SMOKE_ARTIFACT_DIR}/${name}.log"
  bash -lc "${command}" >"${log_file}" 2>&1 &
  SMOKE_LAUNCH_PIDS+=("$!")
  smoke_note "launch_started:${name}"
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
  smoke_wait "node:${node}" "${timeout_s}" "nodes=\"\$(ros2 node list 2>/dev/null || true)\"; grep -qx '${node}' <<<\"\${nodes}\""
}

smoke_wait_topic() {
  local topic="$1" timeout_s="${2:-30}"
  smoke_wait "topic:${topic}" "${timeout_s}" "topics=\"\$(ros2 topic list 2>/dev/null || true)\"; grep -qx '${topic}' <<<\"\${topics}\""
}

smoke_wait_topic_message() {
  local topic="$1" timeout_s="${2:-30}"
  smoke_wait "message:${topic}" "${timeout_s}" "timeout 2 ros2 topic echo '${topic}' --once >/dev/null 2>&1"
}

smoke_wait_lifecycle() {
  local node="$1" timeout_s="${2:-30}"
  smoke_wait "lifecycle:${node}:active" "${timeout_s}" "state=\"\$(ros2 lifecycle get '${node}' 2>/dev/null || true)\"; grep -q active <<<\"\${state}\""
}

smoke_collect_diagnostics() {
  local status="$1"
  ros2 node list >"${SMOKE_ARTIFACT_DIR}/nodes.txt" 2>&1 || true
  ros2 topic list -t >"${SMOKE_ARTIFACT_DIR}/topics.txt" 2>&1 || true
  ros2 service list -t >"${SMOKE_ARTIFACT_DIR}/services.txt" 2>&1 || true
  ros2 param list >"${SMOKE_ARTIFACT_DIR}/parameters.txt" 2>&1 || true
  for node in /collision_monitor /bt_navigator /planner_server /controller_server \
    /keepout_filter_mask_server /route_executor; do
    ros2 lifecycle get "${node}" >"${SMOKE_ARTIFACT_DIR}/lifecycle${node//\//_}.txt" 2>&1 || true
  done
  ros2 topic info /tf -v >"${SMOKE_ARTIFACT_DIR}/tf_publishers.txt" 2>&1 || true
  ros2 topic info /tf_static -v >"${SMOKE_ARTIFACT_DIR}/tf_static_publishers.txt" 2>&1 || true
  python3 - "${SMOKE_ARTIFACT_DIR}/report.json" "${SMOKE_SCENARIO}" \
    "${SMOKE_RUN_ID}" "${SMOKE_STARTED_AT}" "${status}" "${SMOKE_TIMEOUT_S}" \
    "${SMOKE_READY_EVENTS[*]}" <<'PY'
import json
import sys

path, scenario, run_id, started, status, timeout_s, events = sys.argv[1:]
with open(path, "w", encoding="utf-8") as report:
    json.dump({
        "scenario": scenario,
        "run_id": run_id,
        "started_at": started,
        "status": status,
        "smoke_timeout_s": int(timeout_s),
        "readiness": events.split(),
    }, report, indent=2, sort_keys=True)
    report.write("\n")
PY
}

smoke_cleanup() {
  local status=$?
  trap - EXIT
  for pid in "${SMOKE_LAUNCH_PIDS[@]:-}"; do
    kill -TERM "${pid}" 2>/dev/null || true
  done
  for pid in "${SMOKE_LAUNCH_PIDS[@]:-}"; do
    wait "${pid}" 2>/dev/null || true
  done
  smoke_collect_diagnostics "${status}"
  if (( status != 0 )); then
    printf 'Smoke %s failed; diagnostics: %s\n' "${SMOKE_SCENARIO}" "${SMOKE_ARTIFACT_DIR}" >&2
    for log in "${SMOKE_ARTIFACT_DIR}"/*.log; do
      test -e "${log}" && tail -n 100 "${log}" || true
    done
  fi
  return "${status}"
}
