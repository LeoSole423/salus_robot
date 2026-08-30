"""Run a navigation matrix with one fresh obstacle-free simulation per trial."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from .matrix import (EFFECTIVE_SPEED_TOLERANCE_MPS, effective_speed_matches,
                     expand_matrix, matrix_exit_code, write_matrix_artifacts)


def _run(command, *, check=True, capture=False):
    return subprocess.run(command, check=check, text=True, capture_output=capture)


def _wait_ready(timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        result = _run(["timeout", "2", "ros2", "topic", "echo", "/odometry/global",
                       "--once"], check=False, capture=True)
        lifecycle = _run(["timeout", "2", "ros2", "lifecycle", "get",
                          "/controller_server"], check=False, capture=True)
        if result.returncode == 0 and "active" in lifecycle.stdout.lower():
            return
        time.sleep(.25)
    raise RuntimeError(f"navigation readiness timed out after {timeout_s:g}s")


def _failure_bundle(directory, error, metadata):
    directory.mkdir(parents=True, exist_ok=True)
    summary = {"schema_version": 2, "reason": "matrix_setup_failure",
               "terminal_status": None, "errors": [error], "matrix_trial": metadata}
    (directory / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    manifest = {"schema_version": 2, "matrix_trial": metadata}
    (directory / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n"
    )


def _record_metadata(directory, metadata):
    for name in ("manifest.json", "summary.json"):
        path = directory / name
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["matrix_trial"] = metadata
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _speed_metadata(requested_mps, set_result, get_result):
    """Persist auditable set/get output plus the numeric, verified readback."""
    effective, matches = effective_speed_matches(requested_mps, get_result.stdout)
    return {
        "set_returncode": set_result.returncode,
        "set_stdout": set_result.stdout,
        "set_stderr": set_result.stderr,
        "get_returncode": get_result.returncode,
        "get_stdout": get_result.stdout,
        "get_stderr": get_result.stderr,
        "requested_speed_mps": requested_mps,
        "effective_speed_mps": effective,
        "tolerance_mps": EFFECTIVE_SPEED_TOLERANCE_MPS,
        "matches_requested": matches,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", help="strict matrix YAML")
    parser.add_argument("output_dir", help="directory for all trial and matrix artifacts")
    parser.add_argument("--startup-timeout-s", type=float, default=90.0)
    args = parser.parse_args(argv)
    cells = expand_matrix(args.matrix)
    matrix_path, root = Path(args.matrix).resolve(), Path(args.output_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    trial_dirs = []
    outcomes = []
    for cell in cells:
        trial_dir = root / "trials" / cell.trial_id
        trial_dir.parent.mkdir(parents=True, exist_ok=True)
        trial_dirs.append(trial_dir)
        scenario = (matrix_path.parent.parent / cell.case.scenario).resolve()
        metadata = {"matrix_id": cell.matrix_id, "trial_id": cell.trial_id,
                    "repetition": cell.repetition, "requested_speed_mps": cell.speed_mps,
                    "direction": cell.case.direction,
                    "requested_radius_m": cell.case.requested_radius_m,
                    "scenario": str(scenario), "isolation": "fresh_simulation",
                    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
                    "gz_partition": os.environ.get("GZ_PARTITION", "")}
        with (trial_dir.parent / f"{cell.trial_id}-launch.log").open("w") as launch_log:
            launch = subprocess.Popen(
                ["ros2", "launch", "salus_bringup", "integration_sim.launch.py",
                 "capability_profile:=no_obstacle_detection",
                 "world:=/ros2_ws/install/salus_simulation/share/salus_simulation/"
                 "worlds/free.world"], start_new_session=True,
                stdout=launch_log, stderr=subprocess.STDOUT,
            )
            try:
                _wait_ready(args.startup_timeout_s)
                set_result = _run(["ros2", "param", "set", "/controller_server",
                                   "FollowPath.desired_linear_vel", str(cell.speed_mps)],
                                  check=False, capture=True)
                get_result = _run(["ros2", "param", "get", "/controller_server",
                                   "FollowPath.desired_linear_vel"], check=False, capture=True)
                metadata["speed_parameter"] = _speed_metadata(
                    cell.speed_mps, set_result, get_result
                )
                if set_result.returncode != 0 or get_result.returncode != 0:
                    raise RuntimeError("FollowPath.desired_linear_vel runtime update was rejected")
                if not metadata["speed_parameter"]["matches_requested"]:
                    raise RuntimeError(
                        "FollowPath.desired_linear_vel effective readback does not match request"
                    )
                evaluation = _run(
                    ["ros2", "run", "salus_evaluation", "navigation_evaluation", "--ros-args",
                     "-p", "use_sim_time:=true", "-p", "mode:=run", "-p",
                     f"scenario:={scenario}", "-p", f"output_dir:={trial_dir}"],
                    check=False,
                )
                _record_metadata(trial_dir, metadata)
                outcomes.append(
                    "passed" if evaluation.returncode == 0 else "functional_failure"
                )
            except (OSError, RuntimeError, subprocess.CalledProcessError) as exc:
                _failure_bundle(trial_dir, str(exc), metadata)
                outcomes.append("setup_failure")
            finally:
                os.killpg(launch.pid, signal.SIGTERM)
                try:
                    launch.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(launch.pid, signal.SIGKILL)
                    launch.wait()
    write_matrix_artifacts(root / "summary", matrix_path, cells, trial_dirs)
    return matrix_exit_code(outcomes)


if __name__ == "__main__":  # pragma: no cover - console entry point
    raise SystemExit(main())
