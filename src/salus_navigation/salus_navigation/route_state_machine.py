"""Explicit mission transitions; ROS callbacks only request these transitions."""
from .route_model import RouteMission, RoutePhase


def transition(mission: RouteMission, target: RoutePhase, reason: str = "") -> RouteMission:
    allowed = {RoutePhase.IDLE:{RoutePhase.PREPARING}, RoutePhase.PREPARING:{RoutePhase.ACTIVE,RoutePhase.ABORTED}, RoutePhase.ACTIVE:{RoutePhase.PAUSED,RoutePhase.COMPLETED,RoutePhase.CANCELLED,RoutePhase.ABORTED}, RoutePhase.PAUSED:{RoutePhase.CANCELLED,RoutePhase.ABORTED}, RoutePhase.COMPLETED:{RoutePhase.PREPARING}, RoutePhase.CANCELLED:{RoutePhase.PREPARING}, RoutePhase.ABORTED:{RoutePhase.PREPARING}}
    if target not in allowed[mission.phase]: raise ValueError(f"invalid route transition {mission.phase}->{target}")
    mission.phase, mission.pause_reason = target, reason
    return mission
