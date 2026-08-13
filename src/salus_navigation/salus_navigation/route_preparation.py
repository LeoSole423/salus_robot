"""Validation, yaw resolution and expansion; deliberately free of ROS."""
from __future__ import annotations
from math import atan2, degrees, hypot, isfinite
from .route_model import PreparedRoute, RouteWaypoint


def validate_inputs(lats, lons, yaws, actions, roles) -> str:
    if not lats or len(lats) != len(lons) or len(lats) != len(yaws): return "lats, lons and yaws_deg must be non-empty and equally sized"
    if actions and len(actions) != len(lats): return "waypoint_action_jsons length must match lats/lons when provided"
    if roles and len(roles) != len(lats): return "waypoint_roles length must match lats/lons when provided"
    if any(not isfinite(float(v)) for values in (lats, lons) for v in values): return "coordinates must be finite"
    if any(value not in ("", "normal") for value in roles): return "waypoint roles other than normal belong to a future missions cut"
    if any(value.strip() for value in actions): return "waypoint actions belong to a future actions cut"
    return ""


def resolve_yaws(points: list[RouteWaypoint], loop: bool) -> list[RouteWaypoint]:
    result = list(points)
    for index, point in enumerate(result):
        if isfinite(point.yaw_deg): continue
        following = result[(index + 1) % len(result)] if loop or index + 1 < len(result) else point
        yaw = degrees(atan2((following.map_y or 0.0) - (point.map_y or 0.0), (following.map_x or 0.0) - (point.map_x or 0.0)))
        result[index] = RouteWaypoint(**{**point.__dict__, "yaw_deg": yaw})
    return result


def drop_loop_closure(points: list[RouteWaypoint], loop: bool, tolerance_m: float = 0.05) -> list[RouteWaypoint]:
    return points[:-1] if loop and len(points) > 2 and points[0].distance_to(points[-1]) <= tolerance_m else points


def expand(points: list[RouteWaypoint], spacing_m: float, loop: bool) -> list[RouteWaypoint]:
    if spacing_m <= 0.0 or len(points) < 2: return points
    result: list[RouteWaypoint] = []
    pairs = list(zip(points, points[1:] + ([points[0]] if loop else [])))
    for first, second in pairs:
        result.append(first)
        distance = first.distance_to(second); count = int(distance // spacing_m)
        for step in range(1, count + 1):
            fraction = step * spacing_m / distance
            if fraction >= 1.0: break
            result.append(RouteWaypoint(first.lat + (second.lat-first.lat)*fraction, first.lon + (second.lon-first.lon)*fraction, first.yaw_deg, first.input_index, False, map_x=(first.map_x or 0.0)+((second.map_x or 0.0)-(first.map_x or 0.0))*fraction, map_y=(first.map_y or 0.0)+((second.map_y or 0.0)-(first.map_y or 0.0))*fraction))
    return result


def prepare(points: list[RouteWaypoint], *, loop: bool, input_count: int, spacing_m: float, chunk_span_m: float, chunk_max_waypoints: int) -> PreparedRoute:
    points = drop_loop_closure(points, loop)
    points = resolve_yaws(points, loop)
    return PreparedRoute(tuple(expand(points, spacing_m, loop)), loop, input_count, spacing_m, chunk_span_m, chunk_max_waypoints)
