"""Projection only over the active chunk, avoiding false progress elsewhere."""
from math import hypot
from .route_model import RouteChunk, RouteProgress


def project(chunk: RouteChunk, x: float, y: float) -> RouteProgress:
    if not chunk.waypoints: return RouteProgress()
    distances = [hypot((point.map_x or 0.0)-x, (point.map_y or 0.0)-y) for point in chunk.waypoints]
    local = min(range(len(distances)), key=distances.__getitem__)
    point = chunk.waypoints[local]
    return RouteProgress(chunk.start + local, point.input_index if point.key else -1, float(local)/max(1, len(chunk.waypoints)-1), distances[local], distances[-1])
