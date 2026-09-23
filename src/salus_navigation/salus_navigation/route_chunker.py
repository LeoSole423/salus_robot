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


def _legacy_pair_eligible(point) -> bool:
    """Return whether a real point may be the soft first pose of a pair."""
    return bool(
        point.key
        and point.role == "normal"
        and not point.action_json
        and not point.yaw_explicit
    )


def _route_distance(points, start: int, end: int, *, loop: bool) -> float:
    """Return ordered polyline distance from ``start`` through ``end``."""
    if start == end:
        return 0.0
    total = len(points)
    distance = 0.0
    index = start
    while index != end:
        following = (index + 1) % total if loop else index + 1
        if following >= total:
            raise ValueError("route distance reaches past an open route")
        distance += points[index].distance_to(points[following])
        index = following
    return distance


def _next_key_index(route: PreparedRoute, index: int, start: int) -> int | None:
    """Find the next original checkpoint without crossing the active window."""
    total = len(route.waypoints)
    current = index
    for _ in range(total - 1):
        current = (current + 1) % total if route.loop else current + 1
        if not route.loop and current >= total:
            return None
        if current == start:
            return None
        if route.waypoints[current].key:
            return current
    return None


def _build_adaptive_dense_chunk(
    route: PreparedRoute,
    start: int,
    iteration: int,
    *,
    dense_leg_max_m: float,
    dense_horizon_m: float,
) -> RouteChunk | None:
    """Build a legacy-pair window extended across a dense ordered run.

    This is deliberately ordered, not a spatial-clustering operation: only the
    next waypoint in route order can extend a window.  A normal checkpoint may
    be an interior through-pose; hard checkpoints remain terminal boundaries.
    """
    if dense_leg_max_m <= 0.0 or dense_horizon_m <= 0.0:
        raise ValueError("adaptive dense distances must be positive")
    points = route.waypoints
    total = len(points)
    if not points or (not route.loop and start >= total):
        return None

    start %= total
    selected = []
    checkpoint_iterations = []
    index = start
    current_iteration = int(iteration)
    first = points[index]
    first_is_pairable = _legacy_pair_eligible(first)
    key_indices = []

    while True:
        point = points[index]
        selected.append(point)
        if point.key:
            checkpoint_iterations.append(current_iteration)
            key_indices.append(index)
            # Preserve the existing hard-boundary behavior: a hard point is a
            # terminal, never an interior of an adaptive window.
            if len(key_indices) == 1 and not first_is_pairable:
                break
            if len(key_indices) > 1 and not _legacy_pair_eligible(point):
                break

            next_key = _next_key_index(route, index, start)
            if next_key is None:
                break
            # The first two real checkpoints are the legacy-pair baseline.
            # Extending beyond them requires both adjacent legs to be dense
            # and the ordered physical horizon to remain bounded.
            if len(key_indices) >= 2:
                previous_key = key_indices[-2]
                previous_leg = _route_distance(
                    points, previous_key, index, loop=route.loop)
                next_leg = _route_distance(
                    points, index, next_key, loop=route.loop)
                span_to_next = _route_distance(
                    points, start, next_key, loop=route.loop)
                if (
                    previous_leg > dense_leg_max_m
                    or next_leg > dense_leg_max_m
                    or span_to_next > dense_horizon_m
                ):
                    break

        next_index = index + 1
        if route.loop:
            next_index %= total
            if next_index == start:
                break
            # A single NavigateThroughPoses request must never contain a full
            # circuit.  If it did, its terminal pose could coincide with the
            # robot at dispatch and Nav2 could legitimately return success
            # without traversing the intermediate checkpoints.  Leave the
            # final point for the next finite window instead.
            if (next_index + 1) % total == start:
                break
            if next_index == 0 and index != 0:
                current_iteration += 1
        elif next_index >= total:
            break
        index = next_index

    if not any(point.key for point in selected):
        raise ValueError("route chunk has no original checkpoint")
    if not selected[-1].key:
        last_key = max(
            (offset for offset, point in enumerate(selected) if point.key),
            default=-1,
        )
        selected = selected[:last_key + 1]
        checkpoint_iterations = checkpoint_iterations[:len(
            [point for point in selected if point.key]
        )]
    end = ((start + len(selected) - 1) % total if route.loop
           else start + len(selected) - 1)
    return RouteChunk(
        tuple(selected), start, end, iteration, tuple(checkpoint_iterations)
    )


def _build_legacy_pair_chunk(
    route: PreparedRoute, start: int, iteration: int
) -> RouteChunk | None:
    points = route.waypoints
    total = len(points)
    if not points or (not route.loop and start >= total):
        return None
    start %= total
    selected = []
    checkpoint_iterations = []
    index = start
    current_iteration = int(iteration)
    first = points[index]
    first_is_pairable = _legacy_pair_eligible(first)

    while True:
        point = points[index]
        selected.append(point)
        if point.key:
            checkpoint_iterations.append(current_iteration)
            # A hard starting checkpoint is a singleton.  A normal starting
            # checkpoint may continue until the next real checkpoint.
            if len(selected) == 1 and not first_is_pairable:
                break
            if len(selected) > 1 or not first_is_pairable:
                break
        next_index = index + 1
        if route.loop:
            next_index %= total
            if next_index == start:
                break
            if next_index == 0 and index != 0:
                current_iteration += 1
        elif next_index >= total:
            break
        index = next_index

    if not any(point.key for point in selected):
        raise ValueError("route chunk has no original checkpoint")
    if not selected[-1].key:
        # An open route always has a final key, but keep the invariant explicit
        # for malformed prepared routes and loop closure edge cases.
        last_key = max(
            (offset for offset, point in enumerate(selected) if point.key),
            default=-1,
        )
        selected = selected[:last_key + 1]
        checkpoint_iterations = checkpoint_iterations[:1]
    end = (start + len(selected) - 1) % total if route.loop else start + len(selected) - 1
    return RouteChunk(
        tuple(selected), start, end, iteration, tuple(checkpoint_iterations)
    )


def build_chunk(
    route: PreparedRoute,
    start: int,
    iteration: int = 0,
    *,
    mode: str = "single_checkpoint",
    adaptive_dense_leg_max_m: float = 8.0,
    adaptive_dense_horizon_m: float = 35.0,
) -> RouteChunk | None:
    if mode == "legacy_pair":
        return _build_legacy_pair_chunk(route, start, iteration)
    if mode == "adaptive_dense":
        return _build_adaptive_dense_chunk(
            route,
            start,
            iteration,
            dense_leg_max_m=float(adaptive_dense_leg_max_m),
            dense_horizon_m=float(adaptive_dense_horizon_m),
        )
    if mode != "single_checkpoint":
        raise ValueError(
            "mode must be 'single_checkpoint', 'legacy_pair' or 'adaptive_dense'"
        )
    points = route.waypoints; total = len(points)
    if not points or (not route.loop and start >= total): return None
    start %= total; selected = []; index = start
    while not route.loop or len(selected) < max(1, total - 1):
        point = points[index]
        selected.append(point)
        # Every original checkpoint is the terminal of its finite request.
        # Synthetic samples may precede it to provide reach/horizon geometry,
        # but a future checkpoint must never become another hard pose in the
        # same NavigateThroughPoses goal.  This also makes a chunk that starts
        # directly on a key contain exactly that one key.
        if point.key:
            index += 1
            if route.loop:
                index %= total
            break
        index += 1
        if not route.loop and index >= total: break
        index %= total
        # Count/span are soft limits.  Once crossed, retain synthetic geometry
        # until the next original checkpoint so a synthetic point never
        # becomes a success, brake or action boundary.
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


def limit_chunk_to_horizon(
    route: PreparedRoute,
    chunk: RouteChunk,
    *,
    robot_xy: tuple[float, float],
    max_goal_distance_m: float,
) -> RouteChunk | None:
    """End a long request at the last reachable pose before leaving the horizon.

    All poses in the request must fit the conservative radial horizon around
    the dispatch pose. This bounds goals within the rolling global costmap's
    usable interior; a synthetic terminal is only a navigation subgoal.
    """
    limit = float(max_goal_distance_m)
    if not isfinite(limit) or limit <= 0.0:
        raise ValueError("max_goal_distance_m must be finite and positive")
    x, y = map(float, robot_xy)
    if not all(isfinite(value) for value in (x, y)):
        raise ValueError("robot_xy must be finite")
    first_outside = next((
        offset for offset, point in enumerate(chunk.waypoints)
        if _distance_to_waypoint(point, x, y) > limit
    ), None)
    if first_outside is None:
        return chunk
    terminal = first_outside - 1
    if terminal < 0:
        return None
    selected = chunk.waypoints[:terminal + 1]
    key_count = sum(point.key for point in selected)
    total = len(route.waypoints)
    end = (chunk.start + terminal) % total if route.loop else chunk.start + terminal
    return RouteChunk(
        selected, chunk.start, end, chunk.iteration,
        chunk.checkpoint_iterations[:key_count],
    )
