#!/usr/bin/env python3
"""Characterize two independent SALUS simulation universes before matrix parallelism."""

from __future__ import annotations

import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from salus_evaluation.isolation import allocated_trial_isolation, build_trial_env


def _write_json(path, payload):
    Path(path).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n",
                          encoding="utf-8")


def _run(command, env):
    try:
        return subprocess.run(command, env=env, text=True, capture_output=True,
                              timeout=8, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return subprocess.CompletedProcess(command, 124, "", str(exc))


def _lifecycle(env, node):
    result = _run(["python3", "/ros2_ws/tools/ros_lifecycle_probe.py", node], env)
    try:
        state = json.loads(result.stdout).get("state")
    except (TypeError, ValueError, AttributeError):
        state = None
    return {"node": node, "returncode": result.returncode,
            "active": result.returncode == 0 and state == "active",
            "state": state,
            "stdout": result.stdout[-1000:], "stderr": result.stderr[-1000:]}


def _topic_info(worker):
    launch_log = Path(worker["dir"] / "launch.log").read_text(encoding="utf-8")
    marker = "Creating GZ->ROS Bridge: [/world/salus_empty/clock"
    bridge_count = launch_log.count(marker)
    return {"returncode": 0 if bridge_count == 1 else 1,
            "configured_clock_bridge_count": bridge_count,
            "source": "integration_sim_launch_log",
            "stdout": "", "stderr": ""}


def _topic_series(env, clock_samples=5, odometry_samples=3):
    result = _run([
        "python3", "/ros2_ws/tools/ros_topic_probe.py",
        "--clock-samples", str(clock_samples),
        "--odometry-samples", str(odometry_samples),
        "--timeout-s", "8",
    ], env)
    try:
        payload = json.loads(result.stdout)
    except json.JSONDecodeError:
        payload = {}
    payload.update({"returncode": result.returncode, "stderr": result.stderr[-1000:]})
    return payload


def _probe(worker):
    topics = _topic_series(worker["env"], clock_samples=1, odometry_samples=1)
    clock_samples = topics.get("clock_ns", [])
    odometry_samples = topics.get("odometry", [])
    clock = {"returncode": topics.get("returncode", 1),
             "clock_ns": clock_samples[0] if clock_samples else None,
             "stderr": topics.get("stderr", "")}
    odom = {"returncode": topics.get("returncode", 1),
            "sample": odometry_samples[0] if odometry_samples else None,
            "stderr": topics.get("stderr", "")}
    lifecycles = [_lifecycle(worker["env"], node)
                  for node in ("/planner_server", "/controller_server")]
    return {"clock": clock, "odometry": odom, "lifecycles": lifecycles,
            "typed_topic_probe": topics}


def _wait_ready(workers, timeout_s):
    deadline = time.monotonic() + timeout_s
    latest = {}
    while time.monotonic() < deadline:
        for worker in workers:
            latest[worker["id"]] = _probe(worker)
        if all(
            report["clock"]["returncode"] == 0
            and report["clock"]["clock_ns"] is not None
            and report["odometry"]["returncode"] == 0
            and report["odometry"]["sample"] is not None
            and all(item["active"] for item in report["lifecycles"])
            for report in latest.values()
        ):
            return latest
        time.sleep(.25)
    raise RuntimeError(f"readiness timeout: {json.dumps(latest, sort_keys=True)}")


def _terminate(worker):
    process = worker.get("process")
    if process is None:
        return {"pid": None, "term_sent": False, "kill_sent": False,
                "group_alive_after_cleanup": False}
    pgid = process.pid
    term_sent = kill_sent = False
    try:
        os.killpg(pgid, signal.SIGTERM)
        term_sent = True
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
            kill_sent = True
        except ProcessLookupError:
            pass
        process.wait()
    group_alive = True
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            group_alive = False
            break
        time.sleep(.1)
    if group_alive:
        try:
            os.killpg(pgid, signal.SIGKILL)
            kill_sent = True
        except ProcessLookupError:
            group_alive = False
        if group_alive:
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                try:
                    os.killpg(pgid, 0)
                except ProcessLookupError:
                    group_alive = False
                    break
                time.sleep(.1)
    remaining = _group_processes(pgid)
    live_remaining = [item for item in remaining if "Z" not in item["stat"]]
    return {"pid": process.pid, "term_sent": term_sent, "kill_sent": kill_sent,
            "returncode": process.returncode,
            "group_alive_after_cleanup": bool(live_remaining),
            "remaining_processes": remaining,
            "live_processes_after_cleanup": live_remaining}


def _group_processes(pgid):
    result = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,stat="], text=True, capture_output=True,
        check=False,
    )
    processes = []
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) != 3:
            continue
        try:
            pid, process_group = int(fields[0]), int(fields[1])
        except ValueError:
            continue
        if process_group == pgid:
            processes.append({"pid": pid, "stat": fields[2]})
    return processes


def _launch_worker(output, worker_id, isolation):
    worker_dir = Path(output) / worker_id
    isolation.runtime_root.mkdir(parents=True, exist_ok=True)
    isolation.ros_log_dir.mkdir(parents=True, exist_ok=True)
    env = build_trial_env(os.environ, isolation)
    log = (worker_dir / "launch.log").open("w", encoding="utf-8")
    command = ["ros2", "launch", "salus_bringup", "integration_sim.launch.py",
               "capability_profile:=no_obstacle_detection",
               f"zones_runtime_dir:={isolation.runtime_root / 'zones'}"]
    process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    return {"id": worker_id, "dir": worker_dir, "env": env, "isolation": isolation,
            "process": process, "log": log, "started_at": time.time()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir")
    parser.add_argument("--startup-timeout-s", type=float, default=90.0)
    parser.add_argument("--run-token", default=None)
    args = parser.parse_args(argv)
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    run_token = args.run_token or (
        f"isolation-{datetime.now(timezone.utc):%Y%m%dT%H%M%S}-"
        f"{uuid.uuid4().hex[:8]}"
    )
    workers = []
    report = {"schema_version": 1, "status": "failed", "run_token": run_token,
              "workers": {}, "sibling_death": {}, "cleanup": {}}
    allocations = ExitStack()
    try:
        for worker_id in ("worker-a", "worker-b"):
            isolation = allocations.enter_context(allocated_trial_isolation(
                output / worker_id, run_token=run_token, trial_id=worker_id
            ))
            worker = _launch_worker(output, worker_id, isolation)
            workers.append(worker)
            report["workers"][worker_id] = {
                "ros_domain_id": isolation.ros_domain_id,
                "ign_partition": worker["isolation"].partition,
                "gz_partition": worker["isolation"].partition,
                "fastdds_builtin_transports": worker["env"]["FASTDDS_BUILTIN_TRANSPORTS"],
                "runtime_root": str(worker["isolation"].runtime_root),
                "ros_log_dir": str(worker["isolation"].ros_log_dir),
                "process_group_id": worker["process"].pid,
                "started_at_unix_s": worker["started_at"],
            }
        report["concurrent_start"] = {
            "process_group_ids": [worker["process"].pid for worker in workers],
            "all_processes_alive": all(worker["process"].poll() is None
                                        for worker in workers),
        }
        ready = _wait_ready(workers, args.startup_timeout_s)
        for worker in workers:
            info = _topic_info(worker)
            series = _topic_series(worker["env"])
            clock_values = series.get("clock_ns", [])
            odom_values = series.get("odometry", [])
            report["workers"][worker["id"]].update({
                "ready_probe": ready[worker["id"]], "clock_topic_info": info,
                "clock_series": series, "clock_samples": clock_values,
                "clock_monotonic": (
                    series.get("returncode") == 0 and len(clock_values) >= 5
                    and all(a <= b for a, b in zip(clock_values, clock_values[1:]))),
                "odometry_samples": odom_values,
                "odometry_progressive": (
                    series.get("returncode") == 0 and len(odom_values) >= 3
                    and all(a["stamp_ns"] <= b["stamp_ns"]
                            for a, b in zip(odom_values, odom_values[1:]))
                    and any(a["stamp_ns"] < b["stamp_ns"]
                            for a, b in zip(odom_values, odom_values[1:]))),
            })
        report["sibling_death"]["before"] = _probe(workers[1])
        report["sibling_death"]["terminated_worker"] = _terminate(workers[0])
        after_series = _topic_series(workers[1]["env"], clock_samples=5,
                                      odometry_samples=3)
        after_lifecycles = [_lifecycle(workers[1]["env"], node)
                            for node in ("/planner_server", "/controller_server")]
        report["sibling_death"]["worker_b_after"] = {
            "series": after_series, "lifecycles": after_lifecycles,
        }
        report["sibling_death"]["worker_b_survived"] = (
            after_series.get("returncode") == 0
            and after_series.get("clock_complete")
            and after_series.get("odometry_complete")
            and all(a <= b for a, b in zip(
                after_series.get("clock_ns", []), after_series.get("clock_ns", [])[1:]))
            and all(a["stamp_ns"] <= b["stamp_ns"] for a, b in zip(
                after_series.get("odometry", []), after_series.get("odometry", [])[1:]))
            and any(a["stamp_ns"] < b["stamp_ns"] for a, b in zip(
                after_series.get("odometry", []), after_series.get("odometry", [])[1:]))
            and all(state["active"] for state in after_lifecycles)
        )
        report["checks"] = {
            "distinct_domains": len({
                item["ros_domain_id"] for item in report["workers"].values()
            }) == len(workers),
            "distinct_partitions": len({
                item["ign_partition"] for item in report["workers"].values()
            }) == len(workers),
            "matching_partition_names": all(
                item["ign_partition"] == item["gz_partition"]
                for item in report["workers"].values()
            ),
            "clock_sources_isolated": all(
                item["clock_topic_info"]["returncode"] == 0
                and item["clock_topic_info"]["configured_clock_bridge_count"] == 1
                for item in report["workers"].values()
            ),
            "clock_monotonic": all(
                item["clock_monotonic"] for item in report["workers"].values()
            ),
            "odometry_progressive": all(
                item["odometry_progressive"] for item in report["workers"].values()
            ),
            "nav2_active": all(
                all(state["active"] for state in item["ready_probe"]["lifecycles"])
                for item in report["workers"].values()
            ),
            "sibling_death_isolated": report["sibling_death"].get(
                "worker_b_survived", False
            ),
            "workers_alive_simultaneously": report["concurrent_start"][
                "all_processes_alive"
            ],
        }
        report["status"] = "passed" if all(report["checks"].values()) else "failed"
    except Exception as exc:  # retain diagnostics before returning non-zero
        report["error"] = str(exc)
    finally:
        try:
            for worker in reversed(workers):
                cleanup = _terminate(worker)
                worker["log"].close()
                report["cleanup"][worker["id"]] = cleanup
                metadata = report["workers"].get(worker["id"], {})
                _write_json(worker["dir"] / "metadata.json", metadata)
            report["cleanup_complete"] = all(
                not item.get("group_alive_after_cleanup", True)
                for item in report["cleanup"].values()
            )
            if report.get("checks"):
                report["status"] = "passed" if (
                    all(report["checks"].values()) and report["cleanup_complete"]
                ) else "failed"
            _write_json(output / "isolation-report.json", report)
        finally:
            allocations.close()
    return 0 if report["status"] == "passed" and report["cleanup_complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
