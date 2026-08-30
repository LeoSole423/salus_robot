import threading
from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time

from salus_navigation.zones_manager import (
    ZonesManager,
    projected_keepout_state_message,
)


def _projected(zone_id: str, *, enabled: bool = True):
    return {
        "id": zone_id,
        "enabled": enabled,
        "outer_xy": [
            {"x": 1.0, "y": 2.0},
            {"x": 3.0, "y": 2.0},
            {"x": 1.0, "y": 2.0},
        ],
        "holes_xy": [[
            {"x": 1.2, "y": 2.1},
            {"x": 1.4, "y": 2.1},
            {"x": 1.2, "y": 2.1},
        ]],
    }


def test_projected_state_preserves_geometry_and_filters_disabled_zones() -> None:
    message = projected_keepout_state_message(
        [_projected("active"), _projected("disabled", enabled=False)],
        frame_id="map",
        revision=7,
        stamp=Time(sec=12, nanosec=34),
    )

    assert message.header.frame_id == "map"
    assert message.header.stamp.sec == 12
    assert message.header.stamp.nanosec == 34
    assert message.revision == 7
    assert len(message.polygons) == 1

    polygon = message.polygons[0]
    assert polygon.zone_id == "active"
    assert [(point.x, point.y, point.z) for point in polygon.outer.points] == [
        (1.0, 2.0, 0.0),
        (3.0, 2.0, 0.0),
        (1.0, 2.0, 0.0),
    ]
    assert len(polygon.holes) == 1
    assert len(polygon.holes[0].points) == 3


def test_activate_document_advances_revision_and_publishes_once() -> None:
    document = {"type": "FeatureCollection", "features": []}
    projected = [_projected("active")]
    publications = []
    manager = SimpleNamespace(
        _lock=threading.Lock(),
        _document=None,
        _document_text="",
        _mask_ready=False,
        _mask_source="none",
        _projected_revision=4,
        _publish_projected_state=lambda polygons, revision: publications.append(
            (polygons, revision)
        ),
    )

    ZonesManager._activate_document(
        manager,
        document,
        projected,
        mask_source="accepted",
    )

    assert manager._document is document
    assert manager._mask_ready is True
    assert manager._mask_source == "accepted"
    assert manager._projected_revision == 5
    assert publications == [(projected, 5)]


def test_failed_projection_does_not_activate_or_publish_vector_candidate() -> None:
    old_document = {"type": "FeatureCollection", "features": []}
    new_document = {"type": "FeatureCollection", "features": []}
    activations = []

    manager = SimpleNamespace(
        _document=old_document,
        use_keepout=True,
        _project=lambda _document: (None, "projection failed"),
        _activate_document=lambda *args, **kwargs: activations.append((args, kwargs)),
    )

    result = ZonesManager._apply(manager, new_document, persist=True)

    assert result == (False, "projection failed", 0, 0)
    assert not activations


def test_successful_vector_apply_activates_exact_projected_candidate() -> None:
    document = {"type": "FeatureCollection", "features": []}
    candidate = [_projected("accepted")]
    activations = []
    manager = SimpleNamespace(
        _document={"type": "FeatureCollection", "features": []},
        use_keepout=True,
        _project=lambda _document: (candidate, ""),
        _persist_document=lambda _document: (True, ""),
        _activate_document=lambda *args, **kwargs: activations.append((args, kwargs)),
    )

    result = ZonesManager._apply(manager, document, persist=True)

    assert result == (True, "", 0, 0)
    assert len(activations) == 1
    args, kwargs = activations[0]
    assert args == (document, candidate)
    assert kwargs == {"mask_source": "projected_vector_state"}


def test_empty_projected_state_is_explicit_and_revisioned() -> None:
    message = projected_keepout_state_message(
        [],
        frame_id="map",
        revision=1,
        stamp=Time(),
    )

    assert message.header.frame_id == "map"
    assert message.revision == 1
    assert list(message.polygons) == []


def test_multiple_polygons_can_share_one_stable_zone_id() -> None:
    message = projected_keepout_state_message(
        [_projected("multi"), _projected("multi")],
        frame_id="map",
        revision=3,
        stamp=Time(),
    )

    assert [polygon.zone_id for polygon in message.polygons] == ["multi", "multi"]


def test_projected_state_publisher_has_late_subscriber_qos_contract() -> None:
    source = (
        Path(__file__).parents[1]
        / "salus_navigation"
        / "zones_manager.py"
    ).read_text(encoding="utf-8")

    assert '"/zones_manager/projected_keepouts"' in source
    assert "QoSProfile(depth=1)" in source
    assert "ReliabilityPolicy.RELIABLE" in source
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
