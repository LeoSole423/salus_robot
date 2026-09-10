"""Small, explicit per-trial simulation isolation value objects."""

from __future__ import annotations

from dataclasses import dataclass
from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import re


# Keep the evaluation pool below the Linux-safe upper bound requested by the
# harness contract, and away from the domains used by the legacy smoke script.
EVALUATION_DOMAIN_MIN = 64
EVALUATION_DOMAIN_MAX = 79


def _safe_component(value: str) -> str:
    component = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(value)).strip("-")
    return component or "run"


@dataclass(frozen=True)
class TrialIsolation:
    """The host resources that make one simulation a separate universe."""

    ros_domain_id: int
    partition: str
    runtime_root: Path
    ros_log_dir: Path

    def __post_init__(self):
        if not 0 <= int(self.ros_domain_id) <= 232:
            raise ValueError("ros_domain_id must be in the Linux-safe range 0..232")
        if not self.partition:
            raise ValueError("partition must be non-empty")
        if self.runtime_root == self.ros_log_dir:
            raise ValueError("runtime_root and ros_log_dir must be distinct")


def build_trial_env(base_env=None, isolation: TrialIsolation | None = None):
    """Return a child environment without mutating the matrix process environment."""
    if isolation is None:
        raise ValueError("isolation is required")
    environment = dict(os.environ if base_env is None else base_env)
    environment.update({
        "ROS_DOMAIN_ID": str(isolation.ros_domain_id),
        "IGN_PARTITION": isolation.partition,
        "GZ_PARTITION": isolation.partition,
        "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4",
        "ROS_LOG_DIR": str(isolation.ros_log_dir),
    })
    return environment


def make_trial_isolation(root, *, run_token: str, trial_id: str, ros_domain_id: int):
    """Create deterministic paths and an attributable partition for a trial."""
    safe_run = _safe_component(run_token)
    safe_trial = _safe_component(trial_id)
    runtime_root = Path(root) / "runtime" / safe_trial
    return TrialIsolation(
        ros_domain_id=int(ros_domain_id),
        partition=f"salus-nav-{safe_run}-{safe_trial}",
        runtime_root=runtime_root,
        ros_log_dir=runtime_root / "ros-log",
    )


@contextmanager
def allocated_trial_isolation(root, *, run_token: str, trial_id: str,
                              lock_root=None, domain_min=EVALUATION_DOMAIN_MIN,
                              domain_max=EVALUATION_DOMAIN_MAX):
    """Hold one collision-safe domain lock for the complete trial lifetime."""
    if domain_min < 1 or domain_max > 232 or domain_min > domain_max:
        raise ValueError("evaluation domain pool must be a non-empty subset of 1..232")
    configured_lock_root = os.environ.get("SALUS_NAV_EVAL_LOCK_ROOT")
    root_path = (
        Path(lock_root) if lock_root is not None else
        Path(configured_lock_root) if configured_lock_root else
        Path(os.environ.get("TMPDIR", "/tmp")) / "salus-nav-evaluation-domains"
    )
    root_path.mkdir(parents=True, exist_ok=True)
    allocator_path = root_path / "allocator.lock"
    state_path = root_path / "next-domain"
    with allocator_path.open("a+") as allocator:
        fcntl.flock(allocator.fileno(), fcntl.LOCK_EX)
        try:
            try:
                start = int(state_path.read_text(encoding="utf-8"))
            except (FileNotFoundError, ValueError):
                start = domain_min
            if not domain_min <= start <= domain_max:
                start = domain_min
            selected = None
            selected_fd = None
            count = domain_max - domain_min + 1
            for offset in range(count):
                candidate = domain_min + (start - domain_min + offset) % count
                lock_path = root_path / f"domain-{candidate}.lock"
                descriptor = os.open(lock_path, os.O_CREAT | os.O_RDWR, 0o600)
                try:
                    fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    os.close(descriptor)
                    continue
                selected, selected_fd = candidate, descriptor
                next_domain = domain_min + (candidate - domain_min + 1) % count
                state_path.write_text(str(next_domain), encoding="utf-8")
                break
        finally:
            fcntl.flock(allocator.fileno(), fcntl.LOCK_UN)
    if selected is None:
        raise RuntimeError(
            f"no evaluation ROS domain available in {domain_min}..{domain_max}"
        )
    try:
        yield make_trial_isolation(root, run_token=run_token, trial_id=trial_id,
                                   ros_domain_id=selected)
    finally:
        fcntl.flock(selected_fd, fcntl.LOCK_UN)
        os.close(selected_fd)
