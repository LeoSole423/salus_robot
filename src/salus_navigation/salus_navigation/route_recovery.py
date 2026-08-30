"""Deterministic route recovery policy with no ROS dependencies."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import hypot, inf

from .route_model import PreparedRoute


class RecoveryState(str, Enum):
    CLEAR = "CLEAR"
    PENDING = "PENDING"
    WAITING_DATA = "WAITING_DATA"
    WAITING_RETRY = "WAITING_RETRY"
    RECOVERING = "RECOVERING"
    NEEDS_OPERATOR = "NEEDS_OPERATOR"


class RecoveryAction(str, Enum):
    NONE = "NONE"
    CANCEL_AND_BRAKE = "CANCEL_AND_BRAKE"
    BEGIN_RETRY = "BEGIN_RETRY"
    RESUME = "RESUME"


@dataclass(frozen=True)
class RecoveryObservation:
    now_s: float
    path_state: int = 0
    path_reason: str = ""
    nav_failure_code: str = ""
    collision_stop: bool = False
    tf_fresh: bool = True
    costmap_fresh: bool = True
    progress_m: float | None = None


@dataclass(frozen=True)
class RecoveryDecision:
    state: RecoveryState
    action: RecoveryAction = RecoveryAction.NONE
    reason: str = ""
    attempt: int = 0
    wait_remaining_s: float = 0.0


@dataclass(frozen=True)
class ReanchorResolution:
    requested_index: int
    resolved_index: int
    reanchored: bool
    reason: str
    distance_m: float = inf


BLOCKING_FAILURE_CODES = frozenset({
    "NO_VALID_PATH", "CONTROLLER_COLLISION", "COLLISION_STOP",
    "COLLISION_STOP_ACTIVE", "SMOOTHED_PATH_COLLISION", "RECOVERY_FAILED",
    "RECOVERY_OFF_GRID", "COSTMAP_CLEAR_TIMEOUT", "NAV_ABORTED",
})


class BlockedRecoveryPolicy:
    """Classify observations and own the single retry budget for a mission."""

    def __init__(self, *, persistence_s: float = 1.5,
                 retry_wait_s: float = 5.0, max_attempts: int = 3) -> None:
        self.persistence_s = max(0.0, float(persistence_s))
        self.retry_wait_s = max(0.0, float(retry_wait_s))
        self.max_attempts = max(0, int(max_attempts))
        self.reset()

    def reset(self) -> None:
        self.state = RecoveryState.CLEAR
        self.reason = ""
        self.attempt = 0
        self._pending_since: float | None = None
        self._retry_at: float | None = None
        self._resume_state = RecoveryState.CLEAR

    def observe(self, observation: RecoveryObservation) -> RecoveryDecision:
        now = float(observation.now_s)
        if not observation.tf_fresh or not observation.costmap_fresh:
            reason = "TF_STALE" if not observation.tf_fresh else "COSTMAP_STALE"
            if self.state != RecoveryState.WAITING_DATA:
                self._resume_state = self.state
            self.state, self.reason = RecoveryState.WAITING_DATA, reason
            return self._decision(now)

        if self.state == RecoveryState.WAITING_DATA:
            resumed = self._resume_state
            self.state = resumed if resumed != RecoveryState.WAITING_DATA else RecoveryState.CLEAR
            if self.state == RecoveryState.CLEAR:
                self.reason = ""
                return self._decision(now, RecoveryAction.RESUME)

        if self.state in (RecoveryState.NEEDS_OPERATOR, RecoveryState.RECOVERING):
            return self._decision(now)
        if self.state == RecoveryState.WAITING_RETRY:
            if self._retry_at is None or now < self._retry_at:
                return self._decision(now)
            if self.attempt >= self.max_attempts:
                self.state = RecoveryState.NEEDS_OPERATOR
                return self._decision(now)
            self.attempt += 1
            self.state = RecoveryState.RECOVERING
            return self._decision(now, RecoveryAction.BEGIN_RETRY)

        reason = self._blocking_reason(observation)
        if not reason:
            self.state, self.reason, self._pending_since = RecoveryState.CLEAR, "", None
            return self._decision(now)
        if self.state != RecoveryState.PENDING or self.reason != reason:
            self.state, self.reason = RecoveryState.PENDING, reason
            self._pending_since = now
            return self._decision(now)
        pending_since = self._pending_since if self._pending_since is not None else now
        if now - float(pending_since) < self.persistence_s:
            return self._decision(now)
        if self.max_attempts == 0:
            self.state = RecoveryState.NEEDS_OPERATOR
            return self._decision(now, RecoveryAction.CANCEL_AND_BRAKE)
        self.state = RecoveryState.WAITING_RETRY
        self._retry_at = now + self.retry_wait_s
        return self._decision(now, RecoveryAction.CANCEL_AND_BRAKE)

    def finish_retry(self, *, now_s: float, accepted: bool,
                     reason: str = "") -> RecoveryDecision:
        if self.state != RecoveryState.RECOVERING:
            raise RuntimeError("retry result received while recovery is not active")
        if accepted:
            self.state, self.reason, self._retry_at = RecoveryState.CLEAR, "", None
            return self._decision(now_s, RecoveryAction.RESUME)
        self.reason = reason or self.reason or "RETRY_REJECTED"
        if self.attempt >= self.max_attempts:
            self.state = RecoveryState.NEEDS_OPERATOR
        else:
            self.state = RecoveryState.WAITING_RETRY
            self._retry_at = float(now_s) + self.retry_wait_s
        return self._decision(now_s, RecoveryAction.CANCEL_AND_BRAKE)

    def snapshot(self, now_s: float) -> RecoveryDecision:
        """Return observable state without advancing timers or transitions."""
        return self._decision(now_s)

    def _decision(self, now_s: float,
                  action: RecoveryAction = RecoveryAction.NONE) -> RecoveryDecision:
        remaining = 0.0
        if self.state == RecoveryState.WAITING_RETRY and self._retry_at is not None:
            remaining = max(0.0, self._retry_at - float(now_s))
        return RecoveryDecision(self.state, action, self.reason, self.attempt, remaining)

    @staticmethod
    def _blocking_reason(observation: RecoveryObservation) -> str:
        code = str(observation.nav_failure_code or "")
        if code in BLOCKING_FAILURE_CODES:
            return code
        if observation.collision_stop:
            return "COLLISION_STOP_ACTIVE"
        return ""


def resolve_forward_reanchor(route: PreparedRoute, *, current_index: int,
                             robot_x: float | None, robot_y: float | None,
                             tolerance_m: float = 8.0) -> ReanchorResolution:
    """Choose a nearby point without moving backwards in the active lap."""
    points = route.waypoints
    if not points:
        return ReanchorResolution(current_index, current_index, False, "empty_route")
    current = max(0, min(int(current_index), len(points) - 1))
    if robot_x is None or robot_y is None:
        return ReanchorResolution(current, current, False, "pose_unavailable")
    # Do not wrap here. A loop may wrap only after the executor has crossed
    # its closure and advanced loop_iteration.
    candidates = range(current, len(points))
    nearest = min(candidates, key=lambda index:
                  ((points[index].map_x or 0.0) - robot_x) ** 2
                  + ((points[index].map_y or 0.0) - robot_y) ** 2)
    distance = (((points[nearest].map_x or 0.0) - robot_x) ** 2
                + ((points[nearest].map_y or 0.0) - robot_y) ** 2) ** 0.5
    if distance > max(0.0, float(tolerance_m)):
        return ReanchorResolution(current, current, False, "no_forward_match", distance)
    return ReanchorResolution(current, nearest, nearest != current,
                              "forward_match" if nearest != current else "already_anchored",
                              distance)


def checkpoint_within_tolerance(*, checkpoint_x: float | None,
                                checkpoint_y: float | None,
                                robot_x: float | None,
                                robot_y: float | None,
                                tolerance_m: float) -> bool:
    """Return explicit geometric evidence that the pending checkpoint was reached."""
    if None in (checkpoint_x, checkpoint_y, robot_x, robot_y):
        return False
    return hypot(
        float(checkpoint_x) - float(robot_x),
        float(checkpoint_y) - float(robot_y),
    ) <= max(0.0, float(tolerance_m))
