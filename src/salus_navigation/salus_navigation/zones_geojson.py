"""Validation and rasterization helpers for persisted no-go GeoJSON zones."""

from __future__ import annotations

import copy
from typing import Any, Iterable

import cv2
import numpy as np


def normalize_geojson(raw: Any) -> dict[str, Any]:
    """Return a canonical FeatureCollection or raise ValueError."""
    if not isinstance(raw, dict):
        raise ValueError("GeoJSON must be an object")
    kind = raw.get("type")
    if kind == "Feature":
        features = [raw]
    elif kind == "FeatureCollection":
        features = raw.get("features")
        if not isinstance(features, list):
            raise ValueError("FeatureCollection.features must be an array")
    else:
        raise ValueError("GeoJSON must be a Feature or FeatureCollection")
    normalized: list[dict[str, Any]] = []
    for feature_index, feature in enumerate(features):
        if not isinstance(feature, dict) or feature.get("type") != "Feature":
            raise ValueError(f"feature {feature_index} is invalid")
        geometry = feature.get("geometry")
        if not isinstance(geometry, dict) or geometry.get("type") not in ("Polygon", "MultiPolygon"):
            raise ValueError(f"feature {feature_index} must contain Polygon or MultiPolygon geometry")
        properties = copy.deepcopy(feature.get("properties") or {})
        coordinates = geometry.get("coordinates")
        polygons = [coordinates] if geometry["type"] == "Polygon" else coordinates
        if not isinstance(polygons, list):
            raise ValueError(f"feature {feature_index} coordinates are invalid")
        canonical_polygons = [_normalize_polygon(polygon, feature_index) for polygon in polygons]
        normalized.append({
            "type": "Feature",
            "properties": properties,
            "geometry": {
                "type": "Polygon" if len(canonical_polygons) == 1 else "MultiPolygon",
                "coordinates": canonical_polygons[0] if len(canonical_polygons) == 1 else canonical_polygons,
            },
        })
    return {"type": "FeatureCollection", "features": normalized}


def _normalize_polygon(polygon: Any, feature_index: int) -> list[list[list[float]]]:
    if not isinstance(polygon, list) or not polygon:
        raise ValueError(f"feature {feature_index} polygon has no rings")
    return [_normalize_ring(ring, feature_index) for ring in polygon]


def _normalize_ring(ring: Any, feature_index: int) -> list[list[float]]:
    if not isinstance(ring, list) or len(ring) < 3:
        raise ValueError(f"feature {feature_index} polygon ring needs at least three vertices")
    result: list[list[float]] = []
    for point in ring:
        if not isinstance(point, (list, tuple)) or len(point) < 2:
            raise ValueError(f"feature {feature_index} has invalid coordinate")
        lon, lat = float(point[0]), float(point[1])
        if not (-180.0 <= lon <= 180.0):
            raise ValueError("longitude out of range")
        if not (-90.0 <= lat <= 90.0):
            raise ValueError("latitude out of range")
        result.append([lon, lat])
    if result[0] != result[-1]:
        result.append(list(result[0]))
    if len(result) < 4:
        raise ValueError(f"feature {feature_index} polygon ring needs three distinct vertices")
    return result


def feature_and_polygon_counts(document: dict[str, Any]) -> tuple[int, int]:
    return len(document["features"]), sum(len(list(iter_polygons({"type": "FeatureCollection", "features": [f]}))) for f in document["features"])


def iter_polygons(document: dict[str, Any]) -> Iterable[dict[str, Any]]:
    for feature_index, feature in enumerate(document["features"]):
        properties = feature.get("properties") or {}
        geometry = feature["geometry"]
        polygons = [geometry["coordinates"]] if geometry["type"] == "Polygon" else geometry["coordinates"]
        for polygon_index, polygon in enumerate(polygons):
            yield {
                "id": str(properties.get("id") or f"zone_{feature_index}_{polygon_index}"),
                "enabled": bool(properties.get("enabled", True)),
                "outer_ll": polygon[0],
                "holes_ll": polygon[1:],
            }


def rasterize_polygons(
    polygons: list[dict[str, Any]], width: int, height: int, resolution: float,
    origin_x: float, origin_y: float, buffer_margin_m: float,
) -> tuple[np.ndarray, dict[str, int], list[str]]:
    """Render occupied zones (0), free space (255), keeping polygon holes free."""
    image = np.full((height, width), 255, dtype=np.uint8)
    clipped: dict[str, int] = {}
    outside: list[str] = []
    for polygon in polygons:
        if not polygon["enabled"]:
            continue
        outer = _points_to_pixels(polygon["outer_xy"], width, height, resolution, origin_x, origin_y)
        if outer is None:
            outside.append(polygon["id"])
            continue
        points, count = outer
        if count:
            clipped[polygon["id"]] = count
        cv2.fillPoly(image, [np.array(points, dtype=np.int32)], 0)
        for hole in polygon["holes_xy"]:
            converted = _points_to_pixels(
                hole, width, height, resolution, origin_x, origin_y
            )
            if converted is not None:
                hole_points, hole_clipped = converted
                if hole_clipped:
                    clipped[polygon["id"]] = (
                        clipped.get(polygon["id"], 0) + hole_clipped
                    )
                cv2.fillPoly(
                    image,
                    [np.array(hole_points, dtype=np.int32)],
                    255,
                )
    if buffer_margin_m > 0.0:
        radius = max(1, int(round(buffer_margin_m / resolution)))
        occupied = (image == 0).astype(np.uint8)
        occupied = cv2.dilate(occupied, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (radius * 2 + 1, radius * 2 + 1)))
        image[occupied > 0] = 0
    return image, clipped, outside


def _points_to_pixels(points: list[dict[str, float]], width: int, height: int, resolution: float, origin_x: float, origin_y: float):
    xs, ys = [p["x"] for p in points], [p["y"] for p in points]
    if max(xs) < origin_x or min(xs) > origin_x + width * resolution or max(ys) < origin_y or min(ys) > origin_y + height * resolution:
        return None
    output, clipped = [], 0
    for point in points:
        col = int(np.floor((point["x"] - origin_x) / resolution))
        row = int(np.floor((point["y"] - origin_y) / resolution))
        bounded_col, bounded_row = min(width - 1, max(0, col)), min(height - 1, max(0, row))
        clipped += int((bounded_col, bounded_row) != (col, row))
        output.append([bounded_col, height - 1 - bounded_row])
    return output, clipped


def cost_mask_from_binary(image: np.ndarray, resolution: float, radius_m: float, edge_cost: int, min_cost: int) -> np.ndarray:
    """Return Nav2 scale-mask costs: 100 core, exponentially decaying halo."""
    core = image == 0
    costs = np.where(core, 100, 0).astype(np.uint8)
    if radius_m <= 0.0 or not np.any(core):
        return costs
    distance_m = cv2.distanceTransform((~core).astype(np.uint8), cv2.DIST_L2, 3) * resolution
    band = (distance_m > 0.0) & (distance_m <= radius_m)
    decay = np.rint(99.0 * np.exp(-np.log(99.0 / max(1, edge_cost)) * distance_m[band] / radius_m))
    costs[band] = np.clip(decay, min_cost, 99).astype(np.uint8)
    return costs


def scale_image_from_costs(costs: np.ndarray) -> np.ndarray:
    return np.rint((100.0 - np.clip(costs, 0, 100)) * 2.55).astype(np.uint8)
