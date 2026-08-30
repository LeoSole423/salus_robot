"""Run a navigation matrix with one fresh obstacle-free simulation per trial."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory
import yaml

from .matrix import (EFFECTIVE_SPEED_TOLERANCE_MPS, effective_numeric_matches,
                     expand_matrix, matrix_exit_code, write_matrix_artifacts)


def _run(command, *, check=True, capture=False):
    return subprocess.run(command, check=check, text=True, capture_output=capture)


class ReadinessError(RuntimeError):
    """A required runtime resource was not causally ready within the budget."""

    def __init__(self, timeout_s, evidence):
        super().__init__(f"navigation readiness timed out after {timeout_s:g}s")
        self.evidence = evidence


def _lifecycle_active(result):
    return result.returncode == 0 and "active" in result.stdout.lower()


def _readiness_snapshot(require_planner=False):
    """Probe exactly the graph resources the next evaluation operation needs."""
    odometry = _run(
        ["timeout", "2", "ros2", "topic", "echo", "/odometry/global", "--once"],
        check=False, capture=True,
    )
    controller = _run(
        ["timeout", "2", "ros2", "lifecycle", "get", "/controller_server"],
        check=False, capture=True,
    )
    evidence = {
        "odometry_global": {"returncode": odometry.returncode},
        "controller_lifecycle": {
            "returncode": controller.returncode,
            "active": _lifecycle_active(controller),
        },
    }
    ready = odometry.returncode == 0 and _lifecycle_active(controller)
    if not require_planner:
        return ready, evidence
    planner = _run(
        ["timeout", "2", "ros2", "lifecycle", "get", "/planner_server"],
        check=False, capture=True,
    )
    services = _run(
        ["timeout", "2", "ros2", "service", "list"], check=False, capture=True
    )
    planner_get = _run(
        ["timeout", "2", "ros2", "param", "get", "/planner_server", "use_sim_time"],
        check=False, capture=True,
    )
    service_names = set(services.stdout.splitlines())
    get_service = "/planner_server/get_parameters" in service_names
    evidence["planner_lifecycle"] = {
        "returncode": planner.returncode, "active": _lifecycle_active(planner),
    }
    evidence["planner_parameter_services"] = {
        "list_returncode": services.returncode,
        "get_available": get_service, "get_probe_returncode": planner_get.returncode,
    }
    return (
        ready and _lifecycle_active(planner) and get_service
        and planner_get.returncode == 0,
        evidence,
    )


def _wait_ready(timeout_s, *, require_planner=False):
    deadline = time.monotonic() + timeout_s
    evidence = {}
    while time.monotonic() < deadline:
        ready, evidence = _readiness_snapshot(require_planner)
        if ready:
            return
        time.sleep(.25)
    raise ReadinessError(timeout_s, evidence)


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


def _numeric_parameter_metadata(requested, get_result, *, quantity, unit,
                                setup_result=None):
    """Persist an unambiguous numeric parameter set/get exchange."""
    effective, matches = effective_numeric_matches(requested, get_result.stdout)
    return {
        "setup_returncode": (None if setup_result is None else setup_result.returncode),
        "get_returncode": get_result.returncode,
        "get_stdout": get_result.stdout,
        "get_stderr": get_result.stderr,
        f"requested_{quantity}": requested,
        f"effective_{quantity}": effective,
        "unit": unit,
        "tolerance": EFFECTIVE_SPEED_TOLERANCE_MPS,
        "matches_requested": matches,
    }


def write_candidate_nav2_params(base_path, output_path, radius_m):
    """Copy a Nav2 config structurally while changing only Smac's candidate radius."""
    source = Path(base_path)
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    try:
        grid_based = payload["planner_server"]["ros__parameters"]["GridBased"]
    except (KeyError, TypeError) as exc:
        raise ValueError("base Nav2 params lacks planner_server GridBased settings") from exc
    if not isinstance(grid_based, dict):
        raise ValueError("base Nav2 GridBased settings must be a mapping")
    grid_based["minimum_turning_radius"] = float(radius_m)
    output = Path(output_path)
    output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return output


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", help="strict matrix YAML")
    parser.add_argument("output_dir", help="directory for all trial and matrix artifacts")
    parser.add_argument("--startup-timeout-s", type=float, default=90.0)
    parser.add_argument("--planner-minimum-turning-radius", type=float)
    args = parser.parse_args(argv)
    if (args.planner_minimum_turning_radius is not None and
            args.planner_minimum_turning_radius <= 0.0):
        parser.error("--planner-minimum-turning-radius must be positive")
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
        base_params = Path(get_package_share_directory("salus_navigation")) / "config" / (
            "nav2_core_no_obstacles_sim.yaml"
        )
        effective_params = None
        if args.planner_minimum_turning_radius is not None:
            effective_params = write_candidate_nav2_params(
                base_params, trial_dir.parent / f"{cell.trial_id}-nav2.yaml",
                args.planner_minimum_turning_radius,
            )
            metadata["planner_params"] = {
                "base_file": str(base_params), "effective_file": str(effective_params),
                "requested_radius_m": args.planner_minimum_turning_radius,
                "base_sha256": _sha256(base_params),
                "effective_sha256": _sha256(effective_params),
            }
        with (trial_dir.parent / f"{cell.trial_id}-launch.log").open("w") as launch_log:
            launch_args = ["ros2", "launch", "salus_bringup", "integration_sim.launch.py",
                           "capability_profile:=no_obstacle_detection",
                           "world:=/ros2_ws/install/salus_simulation/share/salus_simulation/"
                           "worlds/free.world"]
            if effective_params is not None:
                launch_args.append(f"nav2_no_obstacles_params_file:={effective_params}")
            launch = subprocess.Popen(
                launch_args, start_new_session=True,
                stdout=launch_log, stderr=subprocess.STDOUT,
            )
            try:
                _wait_ready(
                    args.startup_timeout_s,
                    require_planner=args.planner_minimum_turning_radius is not None,
                )
                if args.planner_minimum_turning_radius is not None:
                    get_radius = _run(
                        ["ros2", "param", "get", "/planner_server",
                         "GridBased.minimum_turning_radius"],
                        check=False, capture=True,
                    )
                    metadata["planner_minimum_turning_radius"] = _numeric_parameter_metadata(
                        args.planner_minimum_turning_radius, get_radius,
                        quantity="radius_m", unit="m",
                    )
                    if get_radius.returncode != 0:
                        raise RuntimeError("Smac minimum_turning_radius readback was rejected")
                    if not metadata["planner_minimum_turning_radius"]["matches_requested"]:
                        raise RuntimeError(
                            "Smac effective minimum_turning_radius does not match request"
                        )
                set_result = _run(["ros2", "param", "set", "/controller_server",
                                   "FollowPath.desired_linear_vel", str(cell.speed_mps)],
                                  check=False, capture=True)
                get_result = _run(["ros2", "param", "get", "/controller_server",
                                   "FollowPath.desired_linear_vel"], check=False, capture=True)
                metadata["speed_parameter"] = _numeric_parameter_metadata(
                    cell.speed_mps, get_result, setup_result=set_result,
                    quantity="speed_mps", unit="m/s",
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
                if isinstance(exc, ReadinessError):
                    metadata["readiness"] = exc.evidence
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
