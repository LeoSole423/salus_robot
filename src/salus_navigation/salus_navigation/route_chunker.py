"""Finite route windows whose boundaries are real mission checkpoints."""
from .route_model import PreparedRoute, RouteChunk


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
