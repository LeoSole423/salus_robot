"""Finite route windows.  Loops never hand a complete circuit to Nav2."""
from .route_model import PreparedRoute, RouteChunk


def build_chunk(route: PreparedRoute, start: int, iteration: int = 0) -> RouteChunk | None:
    points = route.waypoints; total = len(points)
    if not points or (not route.loop and start >= total): return None
    start %= total; selected = []; distance = 0.0; index = start
    maximum = max(1, route.chunk_max_waypoints)
    limit = max(0.1, route.chunk_span_m)
    while len(selected) < maximum and (not route.loop or len(selected) < max(1, total-1)):
        point = points[index]
        if selected:
            next_distance = selected[-1].distance_to(point)
            if len(selected) > 1 and distance + next_distance > limit: break
            distance += next_distance
        selected.append(point)
        index += 1
        if not route.loop and index >= total: break
        index %= total
    return RouteChunk(tuple(selected), start, (index-1) % total if route.loop else index-1, iteration)


def next_start(route: PreparedRoute, chunk: RouteChunk) -> int:
    return (chunk.end + 1) % len(route.waypoints) if route.loop else chunk.end + 1
