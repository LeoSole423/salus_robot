"""Pure readiness policy for deterministic Nav2 lifecycle activation."""

from dataclasses import dataclass
from enum import Enum


class StartupState(str, Enum):
    """Observable phases of navigation startup."""

    WAITING_INPUTS = "WAITING_INPUTS"
    STARTING = "STARTING"
    ACTIVE = "ACTIVE"
    FAILED = "FAILED"


@dataclass(frozen=True)
class ReadinessSnapshot:
    """Availability of every prerequisite consumed by Nav2."""

    clock_progressive: bool = False
    odometry_progressive: bool = False
    odometry_finite: bool = False
    transform_available: bool = False
    transform_fresh: bool = False
    scan_valid: bool = False
    scan_fresh: bool = False
    keepout_required: bool = True
    keepout_ready: bool = False
    lifecycle_manager_ready: bool = False


def missing_requirements(snapshot: ReadinessSnapshot) -> tuple[str, ...]:
    """Return stable, machine-readable reasons that prevent activation."""
    checks = (
        (snapshot.clock_progressive, "CLOCK_NOT_PROGRESSIVE"),
        (snapshot.odometry_progressive, "ODOMETRY_NOT_PROGRESSIVE"),
        (snapshot.odometry_finite, "ODOMETRY_INVALID"),
        (snapshot.transform_available, "TF_UNAVAILABLE"),
        (snapshot.transform_fresh, "TF_STALE"),
        (snapshot.scan_valid, "SCAN_INVALID"),
        (snapshot.scan_fresh, "SCAN_STALE"),
        (not snapshot.keepout_required or snapshot.keepout_ready, "KEEPOUT_NOT_READY"),
        (snapshot.lifecycle_manager_ready, "LIFECYCLE_MANAGER_UNAVAILABLE"),
    )
    return tuple(reason for ready, reason in checks if not ready)


class StartupPolicy:
    """Small state machine; ROS transport remains in the coordinator node."""

    def __init__(self) -> None:
        self.state = StartupState.WAITING_INPUTS
        self.reason = "WAITING_FOR_INPUTS"

    def observe(self, snapshot: ReadinessSnapshot) -> bool:
        """Return true once when inputs permit a lifecycle startup request."""
        if self.state is not StartupState.WAITING_INPUTS:
            return False
        missing = missing_requirements(snapshot)
        if missing:
            self.reason = missing[0]
            return False
        self.state = StartupState.STARTING
        self.reason = "LIFECYCLE_START_REQUESTED"
        return True

    def activation_succeeded(self) -> None:
        """Record that all managed nodes reached active."""
        if self.state is StartupState.STARTING:
            self.state = StartupState.ACTIVE
            self.reason = "ALL_NAV2_NODES_ACTIVE"

    def activation_failed(self, reason: str) -> None:
        """Fail terminally; retries require a new process and clean evidence."""
        if self.state is not StartupState.ACTIVE:
            self.state = StartupState.FAILED
            self.reason = str(reason or "LIFECYCLE_START_FAILED")
