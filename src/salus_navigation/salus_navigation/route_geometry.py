"""Small deterministic geometry metrics used by route diagnostics and tests."""
from dataclasses import dataclass
from math import hypot


Point = tuple[float, float]


@dataclass(frozen=True)
class PathGeometryMetrics:
    length_m: float
    direct_distance_m: float
    detour_ratio: float
    max_deviation_m: float
    self_intersections: int


def _distance_to_segment(point: Point, start: Point, end: Point) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    if length_sq <= 1.0e-12:
        return hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
    return hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def _orientation(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: Point, b: Point, c: Point) -> bool:
    return (
        min(a[0], c[0]) - 1.0e-9 <= b[0] <= max(a[0], c[0]) + 1.0e-9
        and min(a[1], c[1]) - 1.0e-9 <= b[1] <= max(a[1], c[1]) + 1.0e-9
    )


def _segments_intersect(first: tuple[Point, Point], second: tuple[Point, Point]) -> bool:
    a, b = first
    c, d = second
    ab_c, ab_d = _orientation(a, b, c), _orientation(a, b, d)
    cd_a, cd_b = _orientation(c, d, a), _orientation(c, d, b)
    if (ab_c > 1.0e-9 and ab_d < -1.0e-9 or ab_c < -1.0e-9 and ab_d > 1.0e-9) and (
        cd_a > 1.0e-9 and cd_b < -1.0e-9 or cd_a < -1.0e-9 and cd_b > 1.0e-9
    ):
        return True
    return any(
        abs(value) <= 1.0e-9 and _on_segment(start, middle, end)
        for value, start, middle, end in (
            (ab_c, a, c, b),
            (ab_d, a, d, b),
            (cd_a, c, a, d),
            (cd_b, c, b, d),
        )
    )


def path_geometry_metrics(
    path: list[Point] | tuple[Point, ...],
    reference: list[Point] | tuple[Point, ...],
) -> PathGeometryMetrics:
    """Measure path cost and topology against the requested route polyline."""
    points = tuple((float(x), float(y)) for x, y in path)
    reference_points = tuple((float(x), float(y)) for x, y in reference)
    segments = tuple(zip(points, points[1:]))
    length = sum(hypot(end[0] - start[0], end[1] - start[1]) for start, end in segments)
    direct = (
        hypot(points[-1][0] - points[0][0], points[-1][1] - points[0][1])
        if len(points) >= 2
        else 0.0
    )
    reference_segments = tuple(zip(reference_points, reference_points[1:]))
    deviation = max(
        (
            min((_distance_to_segment(point, *segment) for segment in reference_segments), default=0.0)
            for point in points
        ),
        default=0.0,
    )
    intersections = sum(
        _segments_intersect(first, second)
        for index, first in enumerate(segments)
        for second in segments[index + 2 :]
        if not set(first).intersection(second)
    )
    return PathGeometryMetrics(
        length_m=length,
        direct_distance_m=direct,
        detour_ratio=(length / direct if direct > 1.0e-9 else 1.0),
        max_deviation_m=deviation,
        self_intersections=intersections,
    )
