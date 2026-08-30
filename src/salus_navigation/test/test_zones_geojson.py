"""Characterization tests for the migrated GeoJSON/keepout representation."""

import hashlib

import numpy as np
import pytest

from salus_navigation.zones_geojson import (
    cost_mask_from_binary,
    feature_and_polygon_counts,
    iter_polygons,
    normalize_geojson,
    rasterize_polygons,
)


def polygon_document(*, enabled=True, hole=False):
    rings = [[[-64.24105, -31.48580], [-64.24100, -31.48580], [-64.24100, -31.48575], [-64.24105, -31.48575]]]
    if hole:
        rings.append([[-64.24104, -31.48579], [-64.24101, -31.48579], [-64.24101, -31.48576], [-64.24104, -31.48576]])
    return {"type": "FeatureCollection", "features": [{"type": "Feature", "properties": {"id": "zone_a", "enabled": enabled}, "geometry": {"type": "Polygon", "coordinates": rings}}]}


def projected(document):
    result = []
    for polygon in iter_polygons(document):
        convert = lambda ring: [{"x": (lon + 64.2410570) * 100000.0, "y": (lat + 31.4858037) * 100000.0} for lon, lat in ring]
        result.append({"id": polygon["id"], "enabled": polygon["enabled"], "outer_xy": convert(polygon["outer_ll"]), "holes_xy": [convert(ring) for ring in polygon["holes_ll"]]})
    return result


def test_normalize_autocloses_ring_and_preserves_disabled_zone():
    document = normalize_geojson(polygon_document(enabled=False))
    assert document["features"][0]["geometry"]["coordinates"][0][0] == document["features"][0]["geometry"]["coordinates"][0][-1]
    assert feature_and_polygon_counts(document) == (1, 1)
    assert list(iter_polygons(document))[0]["enabled"] is False


def test_multipolygon_and_hole_are_supported():
    source = polygon_document(hole=True)
    source["features"][0]["geometry"] = {"type": "MultiPolygon", "coordinates": [source["features"][0]["geometry"]["coordinates"], source["features"][0]["geometry"]["coordinates"]]}
    document = normalize_geojson(source)
    assert feature_and_polygon_counts(document) == (1, 2)
    assert len(list(iter_polygons(document))[0]["holes_ll"]) == 1


def test_invalid_coordinates_are_rejected():
    source = polygon_document()
    source["features"][0]["geometry"]["coordinates"][0][0][1] = -91.0
    with pytest.raises(ValueError, match="latitude out of range"):
        normalize_geojson(source)


def test_rasterize_hole_disabled_buffer_and_outside():
    document = normalize_geojson(polygon_document(hole=True))
    image, clipped, outside = rasterize_polygons(projected(document), 80, 80, 0.1, -2.0, -2.0, 0.0)
    assert not clipped and not outside
    assert image.min() == 0 and image.max() == 255
    # A disabled duplicate leaves the output unchanged.
    disabled = normalize_geojson(polygon_document(enabled=False))
    disabled_image, _, _ = rasterize_polygons(projected(disabled), 80, 80, 0.1, -2.0, -2.0, 0.0)
    assert np.all(disabled_image == 255)
    buffered, _, _ = rasterize_polygons(projected(document), 80, 80, 0.1, -2.0, -2.0, 0.4)
    assert int(np.sum(buffered == 0)) > int(np.sum(image == 0))
    _, _, outside = rasterize_polygons([{"id": "outside", "enabled": True, "outer_xy": [{"x": 99.0, "y": 99.0}] * 4, "holes_xy": []}], 10, 10, 0.1, 0.0, 0.0, 0.0)
    assert outside == ["outside"]


def test_halo_mask_is_deterministic():
    image = np.full((16, 16), 255, dtype=np.uint8); image[7:9, 7:9] = 0
    costs = cost_mask_from_binary(image, 0.1, 0.5, 12, 1)
    assert costs[7, 7] == 100
    assert 0 < costs[6, 7] < 100
    assert costs[0, 0] == 0
    assert hashlib.sha256(costs.tobytes()).hexdigest() == "85d22a9c179b08a2d8c427d925b4db21a3a5e063152b776d5f6ee47337133a6d"



def test_empty_keepout_mask_skips_distance_transform(monkeypatch):
    image = np.full((3000, 3000), 255, dtype=np.uint8)
    calls = []

    def unexpected_distance_transform(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("empty keepout mask must not run distanceTransform")

    monkeypatch.setattr(
        "salus_navigation.zones_geojson.cv2.distanceTransform",
        unexpected_distance_transform,
    )
    costs = cost_mask_from_binary(image, 0.1, 1.5, 12, 1)
    assert not calls
    assert costs.shape == image.shape
    assert costs.dtype == np.uint8
    assert np.count_nonzero(costs) == 0
