"""Run a navigation matrix with one fresh obstacle-free simulation per trial."""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from ament_index_python.packages import get_package_share_directory
import yaml

from .matrix import (EFFECTIVE_SPEED_TOLERANCE_MPS, effective_numeric_matches,
                     expand_matrix, matrix_exit_code, write_matrix_artifacts)
from .isolation import (EVALUATION_DOMAIN_MAX, EVALUATION_DOMAIN_MIN,
                        allocated_trial_isolation, build_trial_env)


def _run(command, *, check=True, capture=False, env=None):
    return subprocess.run(command, check=check, text=True, capture_output=capture,
                          env=env)


class ReadinessError(RuntimeError):
    """A required runtime resource was not causally ready within the budget."""

    def __init__(self, timeout_s, evidence):
        super().__init__(f"navigation readiness timed out after {timeout_s:g}s")
        self.evidence = evidence


def _lifecycle_active(result):
    if result.returncode != 0:
        return False
    try:
        return json.loads(result.stdout).get("state") == "active"
    except (TypeError, ValueError, AttributeError):
        return False


def _readiness_snapshot(require_planner=False, *, env=None):
    """Probe exactly the graph resources the next evaluation operation needs."""
    odometry = _run(
        ["timeout", "8", "ros2", "topic", "echo", "--spin-time", "5",
         "--no-daemon", "/odometry/global", "--once"],
        check=False, capture=True, env=env,
    )
    controller = _run(
        ["timeout", "8", "python3", "/ros2_ws/tools/ros_lifecycle_probe.py",
         "/controller_server"],
        check=False, capture=True, env=env,
    )
    evidence = {
        "odometry_global": {
            "returncode": odometry.returncode,
            "stdout": odometry.stdout[-1000:],
            "stderr": odometry.stderr[-1000:],
        },
        "controller_lifecycle": {
            "returncode": controller.returncode,
            "active": _lifecycle_active(controller),
            "stdout": controller.stdout[-1000:],
            "stderr": controller.stderr[-1000:],
        },
    }
    ready = odometry.returncode == 0 and _lifecycle_active(controller)
    if not require_planner:
        return ready, evidence
    planner = _run(
        ["timeout", "8", "python3", "/ros2_ws/tools/ros_lifecycle_probe.py",
         "/planner_server"],
        check=False, capture=True, env=env,
    )
    services = _run(
        ["timeout", "8", "ros2", "service", "list", "--spin-time", "5",
         "--no-daemon"],
        check=False, capture=True,
        env=env,
    )
    planner_get = _run(
        ["timeout", "8", "python3", "/ros2_ws/tools/ros_parameter_probe.py",
         "get", "/planner_server", "use_sim_time"],
        check=False, capture=True, env=env,
    )
    service_names = set(services.stdout.splitlines())
    get_service = "/planner_server/get_parameters" in service_names
    evidence["planner_lifecycle"] = {
        "returncode": planner.returncode, "active": _lifecycle_active(planner),
        "stdout": planner.stdout[-1000:], "stderr": planner.stderr[-1000:],
    }
    evidence["planner_parameter_services"] = {
        "list_returncode": services.returncode,
        "get_available": get_service, "get_probe_returncode": planner_get.returncode,
        "list_stderr": services.stderr[-1000:], "get_stderr": planner_get.stderr[-1000:],
    }
    return (
        ready and _lifecycle_active(planner) and get_service
        and planner_get.returncode == 0,
        evidence,
    )


def _wait_ready(timeout_s, *, require_planner=False, env=None):
    deadline = time.monotonic() + timeout_s
    evidence = {}
    while time.monotonic() < deadline:
        ready, evidence = _readiness_snapshot(require_planner, env=env)
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


def _trial_result(root, cell):
    """Read an existing matrix outcome without interpreting terminal status."""
    summary_path = Path(root) / "trials" / cell.trial_id / "summary.json"
    try:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return "setup_failure"
    metadata = summary.get("matrix_trial", {})
    return metadata.get("result", "setup_failure") if isinstance(metadata, dict) else (
        "setup_failure"
    )


def _setup_failure_cells(cells, root):
    """Select only existing setup-failure bundles for an explicit rerun."""
    return tuple(cell for cell in cells if _trial_result(root, cell) == "setup_failure")


def _numeric_parameter_metadata(requested, get_result, *, quantity, unit,
                                setup_result=None):
    """Persist an unambiguous numeric parameter set/get exchange."""
    effective, matches = effective_numeric_matches(requested, get_result.stdout)
    return {
        "setup_returncode": (None if setup_result is None else setup_result.returncode),
        "setup_stdout": (None if setup_result is None else setup_result.stdout),
        "setup_stderr": (None if setup_result is None else setup_result.stderr),
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


def _repository_sha():
    """Return the source checkout SHA used by the evaluation container."""
    explicit = os.environ.get("SALUS_NAV_EVAL_SOURCE_SHA", "").strip()
    if explicit:
        return explicit
    roots = [Path("/ros2_ws"), Path.cwd()]
    seen = set()
    for root in roots:
        root = root.resolve()
        if root in seen:
            continue
        seen.add(root)
        result = subprocess.run(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            text=True, capture_output=True, check=False,
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip()
    return "unavailable"


def _resolve_ekf_params_file(matrix_path, configured_path):
    """Resolve a variant path relative to the matrix source file."""
    if configured_path is None:
        return None
    path = Path(configured_path)
    return path.resolve() if path.is_absolute() else (
        Path(matrix_path).parent / path
    ).resolve()


def _build_trial_launch_args(*, zones_runtime_dir, nav2_params_file=None,
                             local_ekf_params_file=None,
                             global_ekf_params_file=None,
                             sim_sensor_profile=None, sim_sensor_seed=None):
    """Build the existing integration launch command plus selected overlays."""
    args = [
        "ros2", "launch", "salus_bringup", "integration_sim.launch.py",
        "capability_profile:=no_obstacle_detection",
        "launch_routes:=true",
        "world:=/ros2_ws/install/salus_simulation/share/salus_simulation/"
        "worlds/free.world",
        f"zones_runtime_dir:={zones_runtime_dir}",
    ]
    if nav2_params_file is not None:
        args.append(f"nav2_no_obstacles_params_file:={nav2_params_file}")
    if sim_sensor_profile is not None:
        args.append(f"sim_sensor_profile:={sim_sensor_profile}")
    if sim_sensor_seed is not None:
        args.append(f"sim_sensor_seed:={sim_sensor_seed}")
    if local_ekf_params_file is not None:
        args.append(f"local_ekf_params_file:={local_ekf_params_file}")
    if global_ekf_params_file is not None:
        args.append(f"global_ekf_params_file:={global_ekf_params_file}")
    return args


def _terminate_process_group(process):
    """Bounded TERM -> KILL cleanup for one owned launch process group."""
    if process is None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline and _live_processes_in_group(process.pid):
        time.sleep(.1)
    if _live_processes_in_group(process.pid):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline and _live_processes_in_group(process.pid):
            time.sleep(.1)


def _live_processes_in_group(pgid):
    result = subprocess.run(
        ["ps", "-eo", "pid=,pgid=,stat="], text=True, capture_output=True,
        check=False,
    )
    for line in result.stdout.splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[1].isdigit() and int(fields[1]) == pgid:
            if "Z" not in fields[2]:
                return True
    return False


DEFAULT_ROUTE_SPACING_M = 1.0


def _trial_metadata(cell, scenario, isolation, source_sha, route_spacing_m=None):
    geometry_variant = os.environ.get(
        "SALUS_NAV_GEOMETRY_VARIANT", "hard_vertex_current"
    ).strip()
    return {
        "matrix_id": cell.matrix_id,
        "trial_id": cell.trial_id,
        "variant": cell.variant_id,
        "evaluation_mode": cell.evaluation_mode,
        "chunk_policy": cell.chunk_policy,
        "local_ekf_params_file": cell.local_ekf_params_file,
        "global_ekf_params_file": cell.global_ekf_params_file,
        "sim_sensor_profile": cell.sim_sensor_profile,
        "sim_sensor_seed": cell.repetition_seed,
        "sim_sensor_seed_base": cell.sim_sensor_seed,
        "source_sha": source_sha,
        "repetition": cell.repetition,
        "requested_speed_mps": cell.speed_mps,
        "direction": cell.case.direction,
        "requested_radius_m": cell.case.requested_radius_m,
        "geometry_variant": geometry_variant,
        "route_spacing_m": (
            None if cell.evaluation_mode != "chunk_continuity"
            else (DEFAULT_ROUTE_SPACING_M if route_spacing_m is None
                  else route_spacing_m)
        ),
        "route_spacing_override_m": route_spacing_m,
        "scenario": str(scenario),
        "isolation": "fresh_simulation",
        "isolation_id": isolation.partition,
        "ros_domain_id": isolation.ros_domain_id,
        "ign_partition": isolation.partition,
        "gz_partition": isolation.partition,
        "fastdds_builtin_transports": "UDPv4",
        "runtime_root": str(isolation.runtime_root),
        "ros_log_dir": str(isolation.ros_log_dir),
    }


def _build_evaluation_command(*, evaluator, scenario, trial_dir, chunk_policy,
                              geometry_variant, evaluation_mode,
                              route_spacing_m=None):
    """Build one evaluator command, with spacing scoped to route evaluation."""
    command = [
        "ros2", "run", "salus_evaluation", evaluator, "--ros-args",
        "-p", "use_sim_time:=true", "-p", "mode:=run", "-p",
        f"scenario:={scenario}", "-p", f"output_dir:={trial_dir}",
        "-p", f"chunk_policy:={chunk_policy}", "-p",
        f"geometry_variant:={geometry_variant}",
    ]
    if evaluation_mode == "chunk_continuity" and route_spacing_m is not None:
        command.extend(["-p", f"route_spacing_m:={route_spacing_m}"])
    return command


def _run_trial_lifecycle(cell, *, matrix_path, trial_dir, startup_timeout_s,
                         planner_minimum_turning_radius, isolation, source_sha,
                         route_spacing_m):
    scenario = (matrix_path.parent.parent / cell.case.scenario).resolve()
    metadata = _trial_metadata(
        cell, scenario, isolation, source_sha, route_spacing_m
    )
    trial_dir.mkdir(parents=True, exist_ok=True)
    isolation.runtime_root.mkdir(parents=True, exist_ok=True)
    isolation.ros_log_dir.mkdir(parents=True, exist_ok=True)
    environment = build_trial_env(os.environ, isolation)
    base_params = Path(get_package_share_directory("salus_navigation")) / "config" / (
        "nav2_core_no_obstacles_sim.yaml"
    )
    effective_params = None
    launch = None
    try:
        planner_config = yaml.safe_load(base_params.read_text(encoding="utf-8"))
        grid_based = planner_config["planner_server"]["ros__parameters"]["GridBased"]
        metadata["planner_contract"] = {
            "params_file": str(base_params),
            "params_sha256": _sha256(base_params),
            "motion_model_for_search": grid_based["motion_model_for_search"],
            "minimum_turning_radius_m": grid_based["minimum_turning_radius"],
            "costmap": "free_world/no_obstacle_detection (unchanged)",
            "steering": "controller profile unchanged",
        }
        if planner_minimum_turning_radius is not None:
            effective_params = write_candidate_nav2_params(
                base_params, trial_dir.parent / f"{cell.trial_id}-nav2.yaml",
                planner_minimum_turning_radius,
            )
            metadata["planner_params"] = {
                "base_file": str(base_params), "effective_file": str(effective_params),
                "requested_radius_m": planner_minimum_turning_radius,
                "base_sha256": _sha256(base_params),
                "effective_sha256": _sha256(effective_params),
            }
        selected_ekf_params = _resolve_ekf_params_file(
            matrix_path, cell.local_ekf_params_file
        )
        if selected_ekf_params is not None:
            if not selected_ekf_params.is_file():
                raise FileNotFoundError(
                    f"selected EKF params file does not exist: {selected_ekf_params}"
                )
            metadata["local_ekf_params_file"] = str(selected_ekf_params)
            metadata["local_ekf_params_sha256"] = _sha256(selected_ekf_params)
        selected_global_ekf_params = _resolve_ekf_params_file(
            matrix_path, cell.global_ekf_params_file
        )
        if selected_global_ekf_params is not None:
            if not selected_global_ekf_params.is_file():
                raise FileNotFoundError(
                    "selected global EKF params file does not exist: "
                    f"{selected_global_ekf_params}"
                )
            metadata["global_ekf_params_file"] = str(selected_global_ekf_params)
            metadata["global_ekf_params_sha256"] = _sha256(selected_global_ekf_params)
        launch_args = _build_trial_launch_args(
            zones_runtime_dir=isolation.runtime_root / "zones",
            nav2_params_file=effective_params,
            local_ekf_params_file=selected_ekf_params,
            global_ekf_params_file=selected_global_ekf_params,
            sim_sensor_profile=cell.sim_sensor_profile,
            sim_sensor_seed=cell.repetition_seed,
        )
        with (trial_dir.parent / f"{cell.trial_id}-launch.log").open("w") as launch_log:
            launch = subprocess.Popen(
                launch_args, env=environment, start_new_session=True,
                stdout=launch_log, stderr=subprocess.STDOUT,
            )
            metadata["process_group_id"] = launch.pid
            try:
                _wait_ready(
                    startup_timeout_s,
                    require_planner=planner_minimum_turning_radius is not None,
                    env=environment,
                )
                if planner_minimum_turning_radius is not None:
                    get_radius = _run(
                        ["timeout", "8", "python3",
                         "/ros2_ws/tools/ros_parameter_probe.py", "get",
                         "/planner_server", "GridBased.minimum_turning_radius"],
                        check=False, capture=True, env=environment,
                    )
                    metadata["planner_minimum_turning_radius"] = _numeric_parameter_metadata(
                        planner_minimum_turning_radius, get_radius,
                        quantity="radius_m", unit="m",
                    )
                    if get_radius.returncode != 0:
                        raise RuntimeError("Smac minimum_turning_radius readback was rejected")
                    if not metadata["planner_minimum_turning_radius"]["matches_requested"]:
                        raise RuntimeError(
                            "Smac effective minimum_turning_radius does not match request"
                        )
                if cell.evaluation_mode == "chunk_continuity":
                    metadata["speed_parameter"] = {
                        "requested_speed_mps": cell.speed_mps,
                        "unit": "m/s",
                        "constant_across_policies": True,
                        "runtime_update": "not_requested",
                    }
                else:
                    set_result = _run(
                        ["timeout", "8", "python3", "/ros2_ws/tools/ros_parameter_probe.py",
                         "set", "/controller_server", "FollowPath.desired_linear_vel",
                         str(cell.speed_mps)],
                        check=False, capture=True, env=environment,
                    )
                    get_result = _run(
                        ["timeout", "8", "python3", "/ros2_ws/tools/ros_parameter_probe.py",
                         "get", "/controller_server", "FollowPath.desired_linear_vel"],
                        check=False, capture=True, env=environment,
                    )
                    metadata["speed_parameter"] = _numeric_parameter_metadata(
                        cell.speed_mps, get_result, setup_result=set_result,
                        quantity="speed_mps", unit="m/s",
                    )
                    if set_result.returncode != 0 or get_result.returncode != 0:
                        raise RuntimeError(
                            "FollowPath.desired_linear_vel runtime update was rejected"
                        )
                    if not metadata["speed_parameter"]["matches_requested"]:
                        raise RuntimeError(
                            "FollowPath.desired_linear_vel effective readback does not "
                            "match request"
                        )
                evaluator = (
                    "navigation_chunk_continuity"
                    if cell.evaluation_mode == "chunk_continuity"
                    else "navigation_evaluation"
                )
                evaluation = _run(
                    _build_evaluation_command(
                        evaluator=evaluator,
                        scenario=scenario,
                        trial_dir=trial_dir,
                        chunk_policy=cell.chunk_policy,
                        geometry_variant=metadata["geometry_variant"],
                        evaluation_mode=cell.evaluation_mode,
                        route_spacing_m=route_spacing_m,
                    ),
                    check=False, env=environment,
                )
                if (trial_dir / "summary.json").exists():
                    trial_summary = json.loads(
                        (trial_dir / "summary.json").read_text(encoding="utf-8")
                    )
                    outcome = (
                        "setup_failure"
                        if trial_summary.get("reason") == "setup_failure"
                        else ("passed" if evaluation.returncode == 0
                              else "functional_failure")
                    )
                    metadata["result"] = outcome
                    _record_metadata(trial_dir, metadata)
                    return outcome
                else:
                    metadata["result"] = "setup_failure"
                    _failure_bundle(
                        trial_dir, "navigation evaluation produced no summary", metadata
                    )
                    return "setup_failure"
            except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
                if isinstance(exc, ReadinessError):
                    metadata["readiness"] = exc.evidence
                metadata["result"] = "setup_failure"
                _failure_bundle(trial_dir, str(exc), metadata)
                return "setup_failure"
            finally:
                _terminate_process_group(launch)
    except (OSError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        metadata["result"] = "setup_failure"
        _failure_bundle(trial_dir, str(exc), metadata)
        return "setup_failure"


def run_trial(cell, *, matrix_path, root, startup_timeout_s,
              planner_minimum_turning_radius, run_token, source_sha,
              route_spacing_m):
    """Run exactly one trial, including identity allocation and cleanup."""
    trial_dir = Path(root) / "trials" / cell.trial_id
    trial_dir.parent.mkdir(parents=True, exist_ok=True)
    try:
        with allocated_trial_isolation(
            trial_dir, run_token=run_token, trial_id=cell.trial_id
        ) as isolation:
            return _run_trial_lifecycle(
                cell, matrix_path=Path(matrix_path), trial_dir=trial_dir,
                startup_timeout_s=startup_timeout_s,
                planner_minimum_turning_radius=planner_minimum_turning_radius,
                isolation=isolation, source_sha=source_sha,
                route_spacing_m=route_spacing_m,
            )
    except Exception as exc:  # preserve the matrix result for allocator/worker errors
        _failure_bundle(trial_dir, str(exc), {
            "matrix_id": cell.matrix_id, "trial_id": cell.trial_id,
            "variant": cell.variant_id,
            "evaluation_mode": cell.evaluation_mode,
            "chunk_policy": cell.chunk_policy,
            "local_ekf_params_file": cell.local_ekf_params_file,
            "global_ekf_params_file": cell.global_ekf_params_file,
            "sim_sensor_profile": cell.sim_sensor_profile,
            "sim_sensor_seed": cell.repetition_seed,
            "sim_sensor_seed_base": cell.sim_sensor_seed,
            "source_sha": source_sha,
            "repetition": cell.repetition, "isolation": "allocation_failure",
            "result": "setup_failure",
        })
        return "setup_failure"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("matrix", help="strict matrix YAML")
    parser.add_argument("output_dir", help="directory for all trial and matrix artifacts")
    parser.add_argument("--startup-timeout-s", type=float, default=90.0)
    parser.add_argument("--planner-minimum-turning-radius", type=float)
    parser.add_argument(
        "--route-spacing-m", type=float,
        help="leg spacing override for chunk_continuity evaluation only",
    )
    parser.add_argument("--jobs", type=int, default=1,
                        help="maximum number of concurrent trials (default: 1)")
    parser.add_argument(
        "--rerun-setup-failures", action="store_true",
        help="rerun only existing trials whose recorded outcome is setup_failure",
    )
    args = parser.parse_args(argv)
    if args.jobs < 1:
        parser.error("--jobs must be a positive integer")
    pool_size = EVALUATION_DOMAIN_MAX - EVALUATION_DOMAIN_MIN + 1
    if args.jobs > pool_size:
        parser.error(f"--jobs cannot exceed the evaluation domain pool size ({pool_size})")
    if (args.planner_minimum_turning_radius is not None and
            args.planner_minimum_turning_radius <= 0.0):
        parser.error("--planner-minimum-turning-radius must be positive")
    if (args.route_spacing_m is not None and
            (not math.isfinite(args.route_spacing_m) or args.route_spacing_m <= 0.0)):
        parser.error("--route-spacing-m must be finite and positive")
    cells = expand_matrix(args.matrix)
    matrix_path, root = Path(args.matrix).resolve(), Path(args.output_dir).resolve()
    root.mkdir(parents=True, exist_ok=True)
    selected_cells = (
        _setup_failure_cells(cells, root) if args.rerun_setup_failures else cells
    )
    run_token = os.environ.get(
        "SALUS_NAV_EVAL_RUN_TOKEN",
        f"matrix-{os.getpid()}-{uuid.uuid4().hex[:8]}",
    )
    arguments = {
        "matrix_path": matrix_path,
        "root": root,
        "startup_timeout_s": args.startup_timeout_s,
        "planner_minimum_turning_radius": args.planner_minimum_turning_radius,
        "run_token": run_token,
        "source_sha": _repository_sha(),
        "route_spacing_m": args.route_spacing_m,
    }
    if args.jobs == 1:
        outcomes = [run_trial(cell, **arguments) for cell in selected_cells]
    else:
        futures = []
        outcomes = []
        with ProcessPoolExecutor(max_workers=args.jobs) as executor:
            for cell in selected_cells:
                futures.append(executor.submit(run_trial, cell, **arguments))
            for cell, future in zip(selected_cells, futures):
                try:
                    outcomes.append(future.result())
                except Exception as exc:
                    trial_dir = root / "trials" / cell.trial_id
                    _failure_bundle(trial_dir, str(exc), {
                        "matrix_id": cell.matrix_id, "trial_id": cell.trial_id,
                        "evaluation_mode": cell.evaluation_mode,
                        "chunk_policy": cell.chunk_policy,
                        "sim_sensor_profile": cell.sim_sensor_profile,
                        "sim_sensor_seed": cell.repetition_seed,
                        "sim_sensor_seed_base": cell.sim_sensor_seed,
                        "route_spacing_m": args.route_spacing_m,
                        "repetition": cell.repetition, "isolation": "worker_exception",
                    })
                    outcomes.append("setup_failure")
    trial_dirs = [root / "trials" / cell.trial_id for cell in cells]
    write_matrix_artifacts(root / "summary", matrix_path, cells, trial_dirs)
    if args.rerun_setup_failures:
        outcomes = [_trial_result(root, cell) for cell in cells]
    return matrix_exit_code(outcomes)


if __name__ == "__main__":  # pragma: no cover - console entry point
    raise SystemExit(main())
