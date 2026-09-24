"""Report large backward excursions in a Nav2 plan relative to a straight chunk."""

from dataclasses import dataclass
from math import hypot


Point = tuple[float, float]


@dataclass(frozen=True)
class PlanRegression:
    backward_m: float
    path_length_m: float
    forward_distance_m: float
    detour_ratio: float


@dataclass
class PlanRegressionTracker:
    active: bool = False

    def observe(self, regression: PlanRegression | None) -> str | None:
        detected = regression is not None
        if detected == self.active:
            return None
        self.active = detected
        return "detected" if detected else "cleared"

    def reset(self) -> None:
        self.active = False


def detect_plan_regression(
    plan: tuple[Point, ...], chunk: tuple[Point, ...], robot: Point,
    *, min_backward_m: float = 6.0, min_detour_ratio: float = 1.5,
) -> PlanRegression | None:
    """Flag substantial backtracking; curved mission chunks are inconclusive.

    Projection uses the chord of an almost straight active chunk. This detects
    the observed U-shaped plans without treating a requested route hairpin as
    a planner defect. It is diagnostic only and never rejects a safe path.
    """
    if len(plan) < 2 or len(chunk) < 2:
        return None
    ax, ay = chunk[0]
    bx, by = chunk[-1]
    direct = hypot(bx - ax, by - ay)
    chunk_length = sum(hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(chunk, chunk[1:]))
    if direct < 1.0 or chunk_length > 1.15 * direct:
        return None
    ux, uy = (bx - ax) / direct, (by - ay) / direct
    backward = max(0.0, max((robot[0] - x) * ux + (robot[1] - y) * uy for x, y in plan))
    path_length = sum(hypot(x2 - x1, y2 - y1) for (x1, y1), (x2, y2) in zip(plan, plan[1:]))
    progress_steps = tuple((x2 - x1) * ux + (y2 - y1) * uy
                           for (x1, y1), (x2, y2) in zip(plan, plan[1:]))
    backward_leg = -sum(min(step, 0.0) for step in progress_steps)
    forward_leg = sum(max(step, 0.0) for step in progress_steps)
    forward = max(0.0, (plan[-1][0] - robot[0]) * ux + (plan[-1][1] - robot[1]) * uy)
    ratio = path_length / max(forward, 1.0)
    enough_backward = backward >= min_backward_m and backward_leg >= min_backward_m
    enough_forward = forward_leg >= min_backward_m
    if not enough_backward or not enough_forward or ratio < min_detour_ratio:
        return None
    return PlanRegression(backward, path_length, forward, ratio)
