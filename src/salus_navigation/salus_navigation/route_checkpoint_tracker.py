"""ROS-free, ordered evidence for intermediate route checkpoints."""
from dataclasses import dataclass
from math import hypot, isfinite


@dataclass(frozen=True)
class CheckpointOccurrence:
    input_index: int
    loop_iteration: int
    x: float
    y: float


@dataclass(frozen=True)
class PoseSample:
    x: float
    y: float
    source_stamp_s: float
    received_steady_s: float


@dataclass(frozen=True)
class CheckpointEvidence:
    accepted: bool
    reason: str
    occurrence: CheckpointOccurrence | None = None
    distance_m: float | None = None


class IntermediateCheckpointTracker:
    """Accept only fresh, ordered, exactly-once soft checkpoint evidence."""

    def __init__(
        self,
        occurrences: tuple[CheckpointOccurrence, ...],
        *,
        radius_m: float = 2.5,
        max_age_s: float = 0.5,
        future_tolerance_s: float = 0.1,
    ) -> None:
        if not isfinite(radius_m) or radius_m <= 0.0:
            raise ValueError("radius_m must be positive and finite")
        if not isfinite(max_age_s) or not 0.0 < max_age_s <= 2.0:
            raise ValueError("max_age_s must be in (0, 2]")
        if not isfinite(future_tolerance_s) or future_tolerance_s < 0.0:
            raise ValueError("future_tolerance_s must be finite and non-negative")
        if any(
            not all(isfinite(value) for value in (item.x, item.y))
            for item in occurrences
        ):
            raise ValueError("checkpoint coordinates must be finite")
        self._occurrences = tuple(occurrences)
        self.radius_m = float(radius_m)
        self.max_age_s = float(max_age_s)
        self.future_tolerance_s = float(future_tolerance_s)
        self._next = 0
        self._last_source_stamp_s = None
        self._minimum_distances = [float("inf")] * len(self._occurrences)

    @property
    def complete(self) -> bool:
        return self._next == len(self._occurrences)

    @property
    def acknowledged(self) -> tuple[CheckpointOccurrence, ...]:
        return self._occurrences[: self._next]

    @property
    def pending(self) -> tuple[CheckpointOccurrence, ...]:
        return self._occurrences[self._next :]

    @property
    def minimum_distances_m(self) -> tuple[float, ...]:
        return tuple(self._minimum_distances)

    def observe(
        self,
        sample: PoseSample,
        *,
        now_ros_s: float,
        now_steady_s: float,
    ) -> CheckpointEvidence:
        values = (
            sample.x,
            sample.y,
            sample.source_stamp_s,
            sample.received_steady_s,
            now_ros_s,
            now_steady_s,
        )
        if not all(isfinite(value) for value in values):
            return CheckpointEvidence(False, "invalid_pose_sample")
        if sample.source_stamp_s <= 0.0:
            return CheckpointEvidence(False, "non_positive_source_stamp")
        if self._last_source_stamp_s is not None and sample.source_stamp_s <= self._last_source_stamp_s:
            return CheckpointEvidence(False, "non_monotonic_source_stamp")
        ros_age = now_ros_s - sample.source_stamp_s
        steady_age = now_steady_s - sample.received_steady_s
        if ros_age < -self.future_tolerance_s or steady_age < 0.0:
            return CheckpointEvidence(False, "future_or_clock_regression")
        if ros_age > self.max_age_s or steady_age > self.max_age_s:
            return CheckpointEvidence(False, "stale_pose")
        self._last_source_stamp_s = sample.source_stamp_s
        if self.complete:
            return CheckpointEvidence(False, "no_pending_checkpoint")
        occurrence = self._occurrences[self._next]
        distance = hypot(sample.x - occurrence.x, sample.y - occurrence.y)
        self._minimum_distances[self._next] = min(
            self._minimum_distances[self._next], distance
        )
        if distance > self.radius_m:
            return CheckpointEvidence(False, "outside_radius", distance_m=distance)
        self._next += 1
        return CheckpointEvidence(
            True,
            "fresh_odometry_radius",
            occurrence=occurrence,
            distance_m=distance,
        )
