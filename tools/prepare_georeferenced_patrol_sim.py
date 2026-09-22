#!/usr/bin/env python3
"""Prepare private, georeferenced simulation artifacts from a saved Cockpit route.

The source route contains operator coordinates and is intentionally written only below
``artifacts/`` (which is ignored by Git).  The companion fixture contains only local
coordinates relative to the selected datum and can be shared for simulation diagnosis.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import yaml


DEFAULT_CONTAINER_ROOT = "/ros2_ws"


class RoutePreparationError(ValueError):
    """Raised when a saved Cockpit route cannot provide a safe temporary datum."""


def _as_finite_coordinate(value: object, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise RoutePreparationError(f"{name} must be numeric") from error
    if not math.isfinite(result):
        raise RoutePreparationError(f"{name} must be finite")
    return result


def _load_routes(path: Path) -> dict[str, Any]:
    raw: object = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(raw, str):
        raw = json.loads(raw)
    if not isinstance(raw, dict):
        raise RoutePreparationError("Cockpit saved-routes export must be an object")
    return raw


def _route_record(routes: dict[str, Any], route_name: str) -> dict[str, Any]:
    record = routes.get(route_name)
    if not isinstance(record, dict):
        raise RoutePreparationError(f"saved route {route_name!r} was not found")
    waypoints = record.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise RoutePreparationError("saved route must contain at least one waypoint")
    if not all(isinstance(waypoint, dict) for waypoint in waypoints):
        raise RoutePreparationError("saved route waypoint entries must be objects")
    return record


def _profile_indices(record: dict[str, Any], count: int) -> dict[str, Any]:
    raw = record.get("patrolMissionProfileRefs")
    if not isinstance(raw, dict):
        raise RoutePreparationError("saved route has no patrol/HOME profile")

    def index(name: str) -> int:
        value = raw.get(name)
        if not isinstance(value, int) or not 0 <= value < count:
            raise RoutePreparationError(f"invalid patrol profile index {name}")
        return value

    def indices(name: str) -> list[int]:
        value = raw.get(name)
        if not isinstance(value, list) or not all(
            isinstance(item, int) and 0 <= item < count for item in value
        ):
            raise RoutePreparationError(f"invalid patrol profile indices {name}")
        return list(value)

    profile = {
        "homeWaypointIndex": index("homeWaypointIndex"),
        "loopWaypointIndices": indices("loopWaypointIndices"),
        "returnWaypointIndices": indices("returnWaypointIndices"),
        "departWaypointIndices": indices("departWaypointIndices"),
        "departEntryWaypointIndex": index("departEntryWaypointIndex"),
    }
    if not profile["loopWaypointIndices"]:
        raise RoutePreparationError("patrol profile must contain a loop")
    return profile


def _local_xy(latitude: float, longitude: float, datum_lat: float, datum_lon: float) -> tuple[float, float]:
    north = (latitude - datum_lat) * 111_320.0
    east = (longitude - datum_lon) * 111_320.0 * math.cos(math.radians(datum_lat))
    return east, north


def _heading_to(source: dict[str, Any], target: dict[str, Any]) -> float:
    source_lat = _as_finite_coordinate(source.get("x"), "datum latitude")
    source_lon = _as_finite_coordinate(source.get("y"), "datum longitude")
    target_lat = _as_finite_coordinate(target.get("x"), "departure latitude")
    target_lon = _as_finite_coordinate(target.get("y"), "departure longitude")
    east, north = _local_xy(target_lat, target_lon, source_lat, source_lon)
    if math.hypot(east, north) <= 1.0e-6:
        raise RoutePreparationError("datum and selected departure waypoint coincide")
    return math.atan2(north, east)


def _write_private(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    os.chmod(path, 0o600)


def _write_world(
    source_world: Path,
    destination: Path,
    latitude: float,
    longitude: float,
    yaw_deg: float,
    plane_size_m: float,
) -> None:
    tree = ET.parse(source_world)
    spherical = tree.find(".//spherical_coordinates")
    if spherical is None:
        raise RoutePreparationError("source world does not contain spherical_coordinates")
    latitude_node = spherical.find("latitude_deg")
    longitude_node = spherical.find("longitude_deg")
    if latitude_node is None or longitude_node is None:
        raise RoutePreparationError("source world has incomplete spherical_coordinates")
    latitude_node.text = f"{latitude:.10f}"
    longitude_node.text = f"{longitude:.10f}"
    heading_node = spherical.find("heading_deg")
    if heading_node is None:
        heading_node = ET.SubElement(spherical, "heading_deg")
    heading_node.text = f"{yaw_deg:.10f}"
    for plane_size_node in tree.findall(
        ".//model[@name='ground_plane']//geometry/plane/size"
    ):
        plane_size_node.text = f"{plane_size_m:.3f} {plane_size_m:.3f}"
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    os.chmod(destination, 0o600)


def _ground_plane_size_m(relative_waypoints: list[dict[str, Any]]) -> float:
    """Return a square ground plane that encloses the route plus a fixed margin."""
    max_extent = max(
        max(abs(float(waypoint["east_m"])), abs(float(waypoint["north_m"])))
        for waypoint in relative_waypoints
    )
    return float(max(200, math.ceil(2.0 * (max_extent + 50.0))))


def prepare_artifacts(
    *,
    routes_file: Path,
    route_name: str,
    source_world: Path,
    output_dir: Path,
    container_root: str = DEFAULT_CONTAINER_ROOT,
    datum_yaw_deg: float = 0.0,
) -> dict[str, Any]:
    """Create the private and sanitized artifacts needed for one georeferenced run."""
    routes = _load_routes(routes_file)
    record = _route_record(routes, route_name)
    waypoints = list(record["waypoints"])
    try:
        profile = _profile_indices(record, len(waypoints))
    except RoutePreparationError:
        profile = None
    if profile is None:
        if len(waypoints) < 2:
            raise RoutePreparationError("generic route needs at least two waypoints")
        datum_index = 0
        departure_index = 1
        datum_kind = "first_waypoint"
    else:
        datum_index = profile["homeWaypointIndex"]
        departure_index = (
            profile["departWaypointIndices"][0]
            if profile["departWaypointIndices"]
            else profile["departEntryWaypointIndex"]
        )
        datum_kind = "home"
    datum = waypoints[datum_index]
    datum_lat = _as_finite_coordinate(datum.get("x"), "datum latitude")
    datum_lon = _as_finite_coordinate(datum.get("y"), "datum longitude")
    datum_yaw_deg = _as_finite_coordinate(datum_yaw_deg, "datum yaw")
    spawn_yaw = _heading_to(datum, waypoints[departure_index])

    output_dir.mkdir(parents=True, exist_ok=False)
    runtime_dir = output_dir / "runtime"
    runtime_dir.mkdir()
    world_path = output_dir / "free_georeferenced_private.world"
    mission_path = output_dir / "patrol_mission_private.json"
    waypoints_path = output_dir / "waypoints_private.yaml"
    sanitized_path = output_dir / "route_fixture_relative.yaml"
    launch_args_path = output_dir / "launch_args_private.txt"
    launcher_path = output_dir / "launch_simulation.sh"

    relative_waypoints = []
    for index, waypoint in enumerate(waypoints):
        east, north = _local_xy(
            _as_finite_coordinate(waypoint.get("x"), "waypoint latitude"),
            _as_finite_coordinate(waypoint.get("y"), "waypoint longitude"),
            datum_lat,
            datum_lon,
        )
        relative_waypoints.append({
            "input_index": index,
            "east_m": round(east, 3),
            "north_m": round(north, 3),
            **({"yaw_deg": float(waypoint["yawDeg"])} if "yawDeg" in waypoint else {}),
            **({"role": waypoint["role"]} if waypoint.get("role") else {}),
            **({"action_types": [action.get("type") for action in waypoint["actions"]]}
               if waypoint.get("actions") else {}),
        })
    ground_plane_size_m = _ground_plane_size_m(relative_waypoints)
    _write_world(
        source_world,
        world_path,
        datum_lat,
        datum_lon,
        datum_yaw_deg,
        ground_plane_size_m,
    )
    _write_private(
        mission_path,
        json.dumps({route_name: record}, ensure_ascii=False, indent=2) + "\n",
    )
    waypoints_document = {
        "waypoints": [
            {
                "latitude": _as_finite_coordinate(waypoint.get("x"), "waypoint latitude"),
                "longitude": _as_finite_coordinate(waypoint.get("y"), "waypoint longitude"),
                **({"yaw": float(waypoint["yawDeg"])} if "yawDeg" in waypoint else {}),
                **({"actions": waypoint["actions"]} if waypoint.get("actions") else {}),
                **({"role": waypoint["role"]} if waypoint.get("role") else {}),
            }
            for waypoint in waypoints
        ],
        **(
            {
                "patrol_profile": {
                    "home_waypoint_index": profile["homeWaypointIndex"],
                    "loop_waypoint_indices": profile["loopWaypointIndices"],
                    "return_waypoint_indices": profile["returnWaypointIndices"],
                    "depart_waypoint_indices": profile["departWaypointIndices"],
                    "depart_entry_waypoint_index": profile["departEntryWaypointIndex"],
                }
            }
            if profile is not None
            else {}
        ),
    }
    _write_private(waypoints_path, yaml.safe_dump(waypoints_document, sort_keys=False))
    sanitized = {
        "route_name": route_name,
        "datum_yaw_deg": datum_yaw_deg,
        "spawn": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": round(spawn_yaw, 6)},
        "datum_kind": datum_kind,
        "datum_input_index": datum_index,
        "ground_plane_size_m": ground_plane_size_m,
        **({"patrol_profile": profile} if profile is not None else {}),
        "waypoints_relative_to_datum": relative_waypoints,
    }
    sanitized_path.write_text(yaml.safe_dump(sanitized, sort_keys=False), encoding="utf-8")

    try:
        relative_output = output_dir.resolve().relative_to(Path.cwd().resolve())
    except ValueError as error:
        raise RoutePreparationError(
            "output-dir must be below the repository so the ROS container can read it"
        ) from error
    container_output = Path(container_root) / relative_output
    private_args = [
        f"world:={container_output / world_path.name}",
        "spawn_x:=0.0",
        "spawn_y:=0.0",
        f"spawn_yaw:={spawn_yaw:.10f}",
        f"datum_lat:={datum_lat:.10f}",
        f"datum_lon:={datum_lon:.10f}",
        f"datum_yaw_deg:={datum_yaw_deg:.10f}",
        f"runtime_dir:={container_output / runtime_dir.name}",
        f"web_waypoints_file:={container_output / waypoints_path.name}",
    ]
    _write_private(launch_args_path, "\n".join(private_args) + "\n")
    _write_private(
        launcher_path,
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        f"repo_dir={Path.cwd().resolve()}\n"
        "cd \"${repo_dir}\"\n"
        "docker compose up -d --build\n"
        "xhost +local:docker >/dev/null\n"
        "trap 'xhost -local:docker >/dev/null 2>&1 || true' EXIT\n"
        "docker compose exec ros2 bash -lc '\n"
        "  set -e\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  cd /ros2_ws\n"
        "  colcon build --symlink-install --packages-up-to salus_bringup\n"
        "  source /ros2_ws/install/setup.bash\n"
        "  exec ros2 launch salus_bringup sim_operational.launch.py rviz:=true $(tr \"\\n\" \" \" < "
        f"{container_output / launch_args_path.name})\n"
        "'\n",
    )
    os.chmod(launcher_path, 0o700)
    return {
        "route_name": route_name,
        "waypoint_count": len(waypoints),
        "datum_index": datum_index,
        "datum_kind": datum_kind,
        "output_dir": output_dir,
        "world_path": world_path,
        "mission_path": mission_path,
        "waypoints_path": waypoints_path,
        "sanitized_path": sanitized_path,
        "launcher_path": launcher_path,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--routes-file", type=Path, required=True)
    parser.add_argument("--route-name", default="PatrullaSencillaPolo")
    parser.add_argument("--source-world", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--container-root", default=DEFAULT_CONTAINER_ROOT)
    parser.add_argument("--datum-yaw-deg", default=0.0, type=float)
    args = parser.parse_args()
    result = prepare_artifacts(
        routes_file=args.routes_file,
        route_name=args.route_name,
        source_world=args.source_world,
        output_dir=args.output_dir,
        container_root=args.container_root,
        datum_yaw_deg=args.datum_yaw_deg,
    )
    print(
        "prepared georeferenced patrol artifacts "
        f"route={result['route_name']!r} waypoints={result['waypoint_count']} "
        f"output={result['output_dir']}"
    )
    print(f"sanitized_fixture={result['sanitized_path']}")
    print(f"launch={result['launcher_path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
