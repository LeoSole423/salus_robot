"""Validation, yaw resolution and expansion; deliberately free of ROS."""
from __future__ import annotations
from math import atan2, cos, degrees, hypot, isfinite, sin
from .route_model import PreparedRoute, RouteWaypoint
from .route_actions import parse_actions


def validate_inputs(lats, lons, yaws, actions, roles) -> str:
    if not lats or len(lats) != len(lons) or len(lats) != len(yaws): return "lats, lons and yaws_deg must be non-empty and equally sized"
    if actions and len(actions) != len(lats): return "waypoint_action_jsons length must match lats/lons when provided"
    if roles and len(roles) != len(lats): return "waypoint_roles length must match lats/lons when provided"
    if any(not isfinite(float(v)) for values in (lats, lons) for v in values): return "coordinates must be finite"
    if any(value not in ("", "normal", "hard") for value in roles): return "waypoint roles must be normal or hard"
    for index, value in enumerate(actions):
        if parse_actions(value, index)[2]: return parse_actions(value, index)[2]
    return ""


def _bearing(start: RouteWaypoint, end: RouteWaypoint) -> float | None:
    dx = float(end.map_x or 0.0) - float(start.map_x or 0.0)
    dy = float(end.map_y or 0.0) - float(start.map_y or 0.0)
    return atan2(dy, dx) if hypot(dx, dy) > 1e-9 else None


def resolve_yaws(
    points: list[RouteWaypoint], loop: bool, *, curve_tangent: bool = False,
) -> list[RouteWaypoint]:
    result = list(points)
    for index, point in enumerate(result):
        if isfinite(point.yaw_deg):
            continue
        if not curve_tangent:
            if index + 1 < len(result) or loop:
                origin, following = point, result[(index + 1) % len(result)]
            else:
                origin, following = result[index - 1] if index else point, point
            yaw = degrees(atan2(
                (following.map_y or 0.0) - (origin.map_y or 0.0),
                (following.map_x or 0.0) - (origin.map_x or 0.0),
            ))
            result[index] = RouteWaypoint(**{**point.__dict__, "yaw_deg": yaw})
            continue
        previous = result[index - 1] if index else (
            result[-1] if loop and len(result) > 1 else None
        )
        following = result[(index + 1) % len(result)] if (
            index + 1 < len(result) or (loop and len(result) > 1)
        ) else None
        incoming = _bearing(previous, point) if previous is not None else None
        outgoing = _bearing(point, following) if following is not None else None
        if incoming is None:
            yaw = outgoing if outgoing is not None else 0.0
        elif outgoing is None:
            yaw = incoming
        else:
            x, y = cos(incoming) + cos(outgoing), sin(incoming) + sin(outgoing)
            yaw = outgoing if hypot(x, y) <= 1e-6 else atan2(y, x)
        yaw = degrees(yaw)
        result[index] = RouteWaypoint(**{**point.__dict__, "yaw_deg": yaw})
    return result


def dispatch_yaws(
    points: tuple[RouteWaypoint, ...],
    *,
    approach_xy: tuple[float, float] | None = None,
    curve_tangent: bool = False,
) -> list[float]:
    """Return Nav2 yaws, preserving route tangents only when requested."""
    yaws = [float(point.yaw_deg) for point in points]
    if curve_tangent or not points:
        return yaws

    first = points[0]
    if approach_xy is not None and not first.yaw_explicit:
        values = (approach_xy[0], approach_xy[1], first.map_x, first.map_y)
        if all(value is not None and isfinite(value) for value in values):
            dx = float(first.map_x) - float(approach_xy[0])
            dy = float(first.map_y) - float(approach_xy[1])
            if hypot(dx, dy) > 1e-9:
                yaws[0] = degrees(atan2(dy, dx))

    if len(points) < 2 or points[-1].yaw_explicit:
        return yaws
    previous, terminal = points[-2], points[-1]
    values = (previous.map_x, previous.map_y, terminal.map_x, terminal.map_y)
    if not all(value is not None and isfinite(value) for value in values):
        return yaws
    dx = float(terminal.map_x) - float(previous.map_x)
    dy = float(terminal.map_y) - float(previous.map_y)
    if hypot(dx, dy) > 1e-9:
        yaws[-1] = degrees(atan2(dy, dx))
    return yaws


def drop_loop_closure(points: list[RouteWaypoint], loop: bool, tolerance_m: float = 0.05) -> list[RouteWaypoint]:
    return points[:-1] if loop and len(points) > 2 and points[0].distance_to(points[-1]) <= tolerance_m else points


def expand(
    points: list[RouteWaypoint], spacing_m: float, loop: bool,
    *, curve_tangent: bool = False,
) -> list[RouteWaypoint]:
    if spacing_m <= 0.0 or len(points) < 2: return points
    result: list[RouteWaypoint] = []
    pairs = list(zip(points, points[1:] + ([points[0]] if loop else [])))
    for first, second in pairs:
        result.append(first)
        distance = first.distance_to(second); count = int(distance // spacing_m)
        for step in range(1, count + 1):
            fraction = step * spacing_m / distance
            if fraction >= 1.0: break
            leg_yaw = (
                degrees(_bearing(first, second) or 0.0)
                if curve_tangent else first.yaw_deg
            )
            result.append(RouteWaypoint(
                first.lat + (second.lat - first.lat) * fraction,
                first.lon + (second.lon - first.lon) * fraction,
                leg_yaw, first.input_index, False,
                map_x=(first.map_x or 0.0)
                + ((second.map_x or 0.0) - (first.map_x or 0.0)) * fraction,
                map_y=(first.map_y or 0.0)
                + ((second.map_y or 0.0) - (first.map_y or 0.0)) * fraction,
            ))
    if not loop:
        # ``pairs`` contributes each segment origin.  Preserve the final
        # original point explicitly so an open route always ends at a real
        # checkpoint rather than at its last synthetic sample.
        result.append(points[-1])
    return result


def prepare(points: list[RouteWaypoint], *, loop: bool, input_count: int, spacing_m: float, chunk_span_m: float, chunk_max_waypoints: int, curve_tangent: bool = False) -> PreparedRoute:
    points = drop_loop_closure(points, loop)
    points = resolve_yaws(points, loop, curve_tangent=curve_tangent)
    return PreparedRoute(
        tuple(expand(points, spacing_m, loop, curve_tangent=curve_tangent)),
        loop, input_count,
        spacing_m, chunk_span_m, chunk_max_waypoints,
        auto_yaw_policy="route_tangent" if curve_tangent else "",
    )
