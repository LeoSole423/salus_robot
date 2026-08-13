"""Choose a forward-only entry into a prepared route."""
from math import hypot
from .route_model import PreparedRoute


def select_anchor(route: PreparedRoute, x: float, y: float, reached_tolerance_m: float = 1.2, segment_tolerance_m: float = 5.0) -> int:
    points = route.waypoints
    distances = [hypot((point.map_x or 0.0)-x, (point.map_y or 0.0)-y) for point in points]
    if not route.loop:
        index = 0
        while index < len(points)-1 and distances[index] <= reached_tolerance_m: index += 1
        # Enter a nearby segment at its *following* point: an open route must
        # never choose a prior index merely because it happens to be closer.
        for left in range(index, len(points)-1):
            ax, ay = points[left].map_x or 0.0, points[left].map_y or 0.0
            bx, by = points[left+1].map_x or 0.0, points[left+1].map_y or 0.0
            dx, dy = bx-ax, by-ay; length2 = dx*dx + dy*dy
            if length2 and 0.0 <= ((x-ax)*dx+(y-ay)*dy)/length2 <= 1.0:
                t = ((x-ax)*dx+(y-ay)*dy)/length2
                if hypot(x-(ax+t*dx), y-(ay+t*dy)) <= segment_tolerance_m: index = max(index, left+1)
        return index
    nearest = min(range(len(points)), key=lambda i: distances[i])
    return (nearest + 1) % len(points) if distances[nearest] <= reached_tolerance_m else nearest
