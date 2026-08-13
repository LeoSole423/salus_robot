"""ROS-free immutable data used by the route executor.

Coordinates in ``map_x/map_y`` are filled during the one-shot mission
conversion.  Keeping input and converted values together makes diagnostics
and replay independent from ROS clients.
"""
from dataclasses import dataclass, field
from enum import Enum
from math import hypot


class RoutePhase(str, Enum):
    IDLE = "IDLE"; PREPARING = "PREPARING"; ACTIVE = "ACTIVE"; PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"; CANCELLED = "CANCELLED"; ABORTED = "ABORTED"


@dataclass(frozen=True)
class RouteWaypoint:
    lat: float; lon: float; yaw_deg: float; input_index: int
    key: bool = True; action_json: str = ""; role: str = "normal"
    map_x: float | None = None; map_y: float | None = None

    def distance_to(self, other: "RouteWaypoint") -> float:
        return hypot((self.map_x or 0.0) - (other.map_x or 0.0), (self.map_y or 0.0) - (other.map_y or 0.0))


@dataclass(frozen=True)
class PreparedRoute:
    waypoints: tuple[RouteWaypoint, ...]; loop: bool; input_count: int
    leg_spacing_m: float; chunk_span_m: float; chunk_max_waypoints: int
    anchor_input_index: int = 0; note: str = ""


@dataclass(frozen=True)
class RouteChunk:
    waypoints: tuple[RouteWaypoint, ...]; start: int; end: int; iteration: int


@dataclass(frozen=True)
class RouteProgress:
    expanded_index: int = -1; checkpoint_index: int = -1
    ratio: float = 0.0; cross_track_error_m: float = 0.0; distance_to_target_m: float = float("inf")


@dataclass
class RouteMission:
    prepared: PreparedRoute | None = None; phase: RoutePhase = RoutePhase.IDLE
    mission_id: str = ""; target_index: int = 0; chunk_id: int = 0; loop_iteration: int = 0
    reached: int = 0; pause_reason: str = ""; progress: RouteProgress = field(default_factory=RouteProgress)
