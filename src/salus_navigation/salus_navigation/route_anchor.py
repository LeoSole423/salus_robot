"""Choose a forward-only entry into a prepared route."""
from dataclasses import dataclass
from math import atan2, degrees, hypot, isfinite

from .route_model import PreparedRoute
from .route_geometry import _distance_to_segment


@dataclass(frozen=True)
class AnchorCandidate:
    """A route segment whose finite projection is close to the robot."""

    segment_start_index: int
    segment_end_index: int
    anchor_index: int
    distance_m: float
    projection: float
    heading_error_deg: float | None

    @property
    def is_forward(self) -> bool:
        # A positive forward component means the route segment lies in the
        # robot's forward half-plane. A perpendicular segment is ambiguous.
        return self.heading_error_deg is not None and self.heading_error_deg < 90.0


@dataclass(frozen=True)
class AnchorSelection:
    """Pure anchor decision plus the nearby segment evidence."""

    anchor_index: int | None
    reason: str
    candidates: tuple[AnchorCandidate, ...] = ()

    @property
    def accepted(self) -> bool:
        return self.anchor_index is not None


def _select_open_anchor(
    route: PreparedRoute,
    x: float,
    y: float,
    reached_tolerance_m: float,
    segment_tolerance_m: float,
) -> int:
    """Keep the established forward-only selection rules for open routes."""
    points = route.waypoints
    distances = [hypot((point.map_x or 0.0)-x, (point.map_y or 0.0)-y) for point in points]
    index = 0
    while index < len(points)-1 and distances[index] <= reached_tolerance_m:
        index += 1
    for left in range(index, len(points)-1):
        ax, ay = points[left].map_x or 0.0, points[left].map_y or 0.0
        bx, by = points[left+1].map_x or 0.0, points[left+1].map_y or 0.0
        dx, dy = bx-ax, by-ay
        length2 = dx*dx + dy*dy
        if length2 and 0.0 <= ((x-ax)*dx+(y-ay)*dy)/length2 <= 1.0:
            t = ((x-ax)*dx+(y-ay)*dy)/length2
            if hypot(x-(ax+t*dx), y-(ay+t*dy)) <= segment_tolerance_m:
                index = max(index, left+1)
    return index


def select_anchor(
    route: PreparedRoute,
    x: float,
    y: float,
    reached_tolerance_m: float = 1.2,
    segment_tolerance_m: float = 5.0,
    *,
    heading_deg: float | None = None,
) -> AnchorSelection:
    """Choose a route entry using proximity, segment direction and robot yaw.

    Open routes retain their previous position-only behavior. A fresh loop
    start needs a finite heading and either a nearby segment with a finite
    projection or an outgoing segment at a nearby waypoint. Candidates behind
    or perpendicular to the heading cannot win by being closer. Among forward
    candidates, a segment must be no worse in both lateral distance and
    heading difference to dominate another. If the criteria trade off, the
    choice is ambiguous and rejected.
    """
    if not route.loop:
        return AnchorSelection(
            _select_open_anchor(
                route, x, y, reached_tolerance_m, segment_tolerance_m,
            ),
            "open_route",
        )

    points = route.waypoints
    if not points:
        return AnchorSelection(None, "empty_route")
    heading_valid = heading_deg is not None and isfinite(float(heading_deg))

    candidate_by_segment: dict[tuple[int, int], AnchorCandidate] = {}
    for left, first in enumerate(points):
        right = (left + 1) % len(points)
        second = points[right]
        start = (first.map_x or 0.0, first.map_y or 0.0)
        end = (second.map_x or 0.0, second.map_y or 0.0)
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1.0e-12:
            continue
        projection = ((x - start[0]) * dx + (y - start[1]) * dy) / length_sq
        if not 0.0 <= projection <= 1.0:
            continue
        distance = _distance_to_segment((x, y), start, end)
        if distance > segment_tolerance_m:
            continue
        heading_error = None
        if heading_valid:
            bearing_deg = degrees(atan2(dy, dx))
            heading_error = abs(
                (bearing_deg - float(heading_deg) + 180.0) % 360.0 - 180.0
            )
        candidate_by_segment[(left, right)] = AnchorCandidate(
            left, right, right, distance, projection, heading_error,
        )

    # Preserve loop entry near a checkpoint when the robot is just beyond
    # the finite projection of its outgoing segment.
    for left, point in enumerate(points):
        right = (left + 1) % len(points)
        start = (point.map_x or 0.0, point.map_y or 0.0)
        distance = hypot(x - start[0], y - start[1])
        if distance > reached_tolerance_m:
            continue
        next_point = points[right]
        dx = (next_point.map_x or 0.0) - start[0]
        dy = (next_point.map_y or 0.0) - start[1]
        if hypot(dx, dy) <= 1.0e-12:
            continue
        heading_error = None
        if heading_valid:
            bearing_deg = degrees(atan2(dy, dx))
            heading_error = abs(
                (bearing_deg - float(heading_deg) + 180.0) % 360.0 - 180.0
            )
        candidate = AnchorCandidate(
            left, right, right, distance, 0.0, heading_error,
        )
        key = (left, right)
        current = candidate_by_segment.get(key)
        if current is None or candidate.distance_m < current.distance_m:
            candidate_by_segment[key] = candidate

    candidates = list(candidate_by_segment.values())

    if not heading_valid:
        return AnchorSelection(None, "orientation_unavailable", tuple(candidates))
    if not candidates:
        return AnchorSelection(None, "no_near_segment")
    forward = [candidate for candidate in candidates if candidate.is_forward]
    if not forward:
        return AnchorSelection(None, "no_forward_segment", tuple(candidates))

    best_candidates = [candidate for candidate in forward if not any(
        other is not candidate
        and other.heading_error_deg is not None
        and candidate.heading_error_deg is not None
        and other.distance_m <= candidate.distance_m
        and other.heading_error_deg <= candidate.heading_error_deg
        and (
            other.distance_m < candidate.distance_m
            or other.heading_error_deg < candidate.heading_error_deg
        )
        for other in forward
    )]
    if len(best_candidates) != 1:
        return AnchorSelection(None, "ambiguous_forward_segments", tuple(candidates))
    return AnchorSelection(
        best_candidates[0].anchor_index, "best_forward_segment", tuple(candidates),
    )
