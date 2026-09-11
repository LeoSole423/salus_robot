"""Finite route windows whose boundaries are real mission checkpoints."""
from dataclasses import dataclass
from math import hypot, isfinite

from .route_model import PreparedRoute, RouteChunk


@dataclass(frozen=True)
class DispatchStart:
    """Forward-only start resolution applied before every chunk dispatch."""

    index: int | None
    skipped_reached: int = 0
    skipped_synthetic: int = 0


def _distance_to_waypoint(point, x: float, y: float) -> float:
    return hypot((point.map_x or 0.0) - x, (point.map_y or 0.0) - y)


def _passed_synthetic_segment(start, end, x: float, y: float, tolerance: float) -> bool:
    ax, ay = float(start.map_x or 0.0), float(start.map_y or 0.0)
    bx, by = float(end.map_x or 0.0), float(end.map_y or 0.0)
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-9:
        return False
    ratio = ((x - ax) * dx + (y - ay) * dy) / length_sq
    if ratio <= 0.0:
        return False
    ratio = min(1.0, ratio)
    projected_x, projected_y = ax + ratio * dx, ay + ratio * dy
    return hypot(x - projected_x, y - projected_y) <= tolerance


def resolve_dispatch_start(
    route: PreparedRoute,
    start: int,
    *,
    robot_xy: tuple[float, float] | None,
    reached_tolerance_m: float,
    synthetic_segment_tolerance_m: float,
    protected_indices: set[int] | None = None,
) -> DispatchStart:
    """Resolve a chunk start without moving backwards or skipping actions.

    This is the small forward-only equivalent of the legacy executor's two
    pre-dispatch guards.  Reached waypoints are skipped by radius, then a
    synthetic point is skipped only when the robot is already beyond its
    outgoing segment.  Action checkpoints are protected even when they are
    within the reached tolerance.
    """
    points = route.waypoints
    total = len(points)
    if not points:
        return DispatchStart(None)
    requested = max(0, int(start))
    if not route.loop and requested >= total:
        return DispatchStart(None)
    current = requested % total if route.loop else requested
    if robot_xy is None or not all(isfinite(float(value)) for value in robot_xy):
        return DispatchStart(current)

    x, y = map(float, robot_xy)
    protected = {int(index) for index in (protected_indices or set())}
    protected.update(
        index for index, point in enumerate(points) if point.key and point.action_json
    )
    reached_tolerance = max(0.05, float(reached_tolerance_m))
    segment_tolerance = max(0.05, float(synthetic_segment_tolerance_m))
    max_steps = max(0, total - 1)
    skipped_reached = 0
    # Every original checkpoint is an observable mission boundary. Patrol
    # consumes ROUTE_CHECKPOINT_REACHED, including the EXIT_LOOP checkpoint,
    # so pre-dispatch pruning may only skip synthetic geometry.
    while (
        skipped_reached < max_steps
        and current not in protected
        and not points[current].key
    ):
        if _distance_to_waypoint(points[current], x, y) > reached_tolerance:
            break
        next_index = current + 1
        if route.loop:
            next_index %= total
        elif next_index >= total:
            current = total
            break
        current = next_index
        skipped_reached += 1

    skipped_synthetic = 0
    while (
        current < total
        and skipped_synthetic < max_steps
        and not points[current].key
        and current not in protected
    ):
        next_index = current + 1
        if route.loop:
            next_index %= total
        elif next_index >= total:
            break
        if next_index == current or not _passed_synthetic_segment(
            points[current], points[next_index], x, y, segment_tolerance
        ):
            break
        current = next_index
        skipped_synthetic += 1

    return DispatchStart(current, skipped_reached, skipped_synthetic)


def build_chunk(route: PreparedRoute, start: int, iteration: int = 0) -> RouteChunk | None:
    points = route.waypoints; total = len(points)
    if not points or (not route.loop and start >= total): return None
    start %= total; selected = []; distance = 0.0; index = start
    maximum = max(1, route.chunk_max_waypoints)
    limit = max(0.1, route.chunk_span_m)
    limit_reached = False
    while not route.loop or len(selected) < max(1, total - 1):
        point = points[index]
        if selected:
            next_distance = selected[-1].distance_to(point)
            distance += next_distance
        selected.append(point)
        # Programmed actions are hard mission boundaries.  They execute only
        # after Nav2 has completed the finite chunk ending at that checkpoint.
        if point.key and point.action_json and len(selected) >= 1:
            index += 1
            if route.loop:
                index %= total
            break
        # Match the physically validated legacy contract: a finite chunk may
        # contain every synthetic sample along one leg, but it ends at the
        # next original checkpoint.  Sending several original checkpoints in
        # one NavigateThroughPoses goal forces the Dubins planner to satisfy
        # several independent headings at once and can create large loops on
        # otherwise short route legs.
        if point.key and len(selected) > 1:
            index += 1
            if route.loop:
                index %= total
            break
        limit_reached = len(selected) >= maximum or distance >= limit
        index += 1
        if not route.loop and index >= total: break
        index %= total
        # Count/span are soft limits.  Once crossed, retain synthetic geometry
        # until the next original checkpoint so a synthetic point never
        # becomes a success, brake or action boundary.
        if limit_reached and point.key and len(selected) > 1:
            break
    if route.loop and selected and not selected[-1].key:
        last_checkpoint = max(
            (offset for offset, point in enumerate(selected) if point.key),
            default=-1,
        )
        selected = selected[:last_checkpoint + 1]
        index = (start + len(selected)) % total
    if selected and not any(point.key for point in selected):
        raise ValueError("route chunk has no original checkpoint")
    return RouteChunk(tuple(selected), start, (index-1) % total if route.loop else index-1, iteration)


def next_start(route: PreparedRoute, chunk: RouteChunk) -> int:
    return (chunk.end + 1) % len(route.waypoints) if route.loop else chunk.end + 1
