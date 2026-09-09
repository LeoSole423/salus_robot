"""Projection only over active chunk segments, avoiding false progress elsewhere."""
from math import hypot
from .route_model import RouteChunk, RouteProgress


def project(chunk: RouteChunk, x: float, y: float) -> RouteProgress:
    points = chunk.waypoints
    if not points:
        return RouteProgress()
    if len(points) == 1:
        point = points[0]
        distance = hypot((point.map_x or 0.0) - x, (point.map_y or 0.0) - y)
        return RouteProgress(
            chunk.start,
            point.input_index if point.key else -1,
            1.0,
            distance,
            distance,
        )

    lengths = [points[index].distance_to(points[index + 1]) for index in range(len(points) - 1)]
    total = sum(lengths)
    travelled = 0.0
    best = None
    for local, length in enumerate(lengths):
        first, second = points[local], points[local + 1]
        ax, ay = first.map_x or 0.0, first.map_y or 0.0
        bx, by = second.map_x or 0.0, second.map_y or 0.0
        if length <= 1e-9:
            fraction = 0.0
        else:
            fraction = max(0.0, min(1.0, ((x - ax) * (bx - ax) + (y - ay) * (by - ay)) / (length * length)))
        projected_x = ax + fraction * (bx - ax)
        projected_y = ay + fraction * (by - ay)
        distance = hypot(x - projected_x, y - projected_y)
        candidate = (distance, travelled + fraction * length, local, fraction)
        if best is None or candidate[:2] < best[:2]:
            best = candidate
        travelled += length

    distance, along, local, fraction = best
    expanded_local = local + (1 if fraction >= 1.0 else 0)
    checkpoint_index = -1
    for point in points[:expanded_local + 1]:
        if point.key:
            checkpoint_index = point.input_index
    return RouteProgress(
        chunk.start + expanded_local,
        checkpoint_index,
        1.0 if total <= 1e-9 else along / total,
        distance,
        hypot((points[-1].map_x or 0.0) - x, (points[-1].map_y or 0.0) - y),
    )
