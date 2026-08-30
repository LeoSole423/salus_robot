import threading
from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time

from salus_navigation.nav2_startup_coordinator import (
    Nav2StartupCoordinator,
    zones_state_mask_ready,
)
from salus_navigation.zones_manager import (
    ZonesManager,
    projected_keepout_state_message,
    unrepresentable_zone_error,
    zones_document_is_empty,
)


ROOT = Path(__file__).parents[1]
SOURCE = ROOT / "salus_navigation" / "nav2_startup_coordinator.py"
ZONES_SOURCE = ROOT / "salus_navigation" / "zones_manager.py"


def test_zones_state_is_required_to_confirm_active_keepout_generation() -> None:
    assert not zones_state_mask_ready(None)
    assert not zones_state_mask_ready(SimpleNamespace(ok=False, mask_ready=True))
    assert not zones_state_mask_ready(SimpleNamespace(ok=True, mask_ready=False))
    assert zones_state_mask_ready(SimpleNamespace(ok=True, mask_ready=True))


def test_placeholder_mask_observation_does_not_mark_keepout_ready() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    callback = source[
        source.index("    def _on_mask("):
        source.index("    def _poll_zones_state(")
    ]

    assert "_mask_observed_valid" in callback
    assert "_mask_ready =" not in callback
    assert 'GetZonesState, "/zones_manager/get_state"' in source
    assert "self._poll_zones_state(now)" in source


def test_confirmed_empty_zone_document_can_reuse_empty_bootstrap() -> None:
    assert zones_document_is_empty({"type": "FeatureCollection", "features": []})
    assert not zones_document_is_empty({
        "type": "FeatureCollection",
        "features": [{"type": "Feature"}],
    })


def test_unrepresentable_enabled_zone_is_rejected_explicitly() -> None:
    assert unrepresentable_zone_error({}, []) == ""
    error = unrepresentable_zone_error(
        {"zone_clipped": 2},
        ["zone_outside"],
    )
    assert "legacy fixed mask" in error
    assert "outside=zone_outside" in error
    assert "clipped=zone_clipped:2" in error


def test_zone_mask_rejects_unrepresentable_geometry_before_staging() -> None:
    source = ZONES_SOURCE.read_text(encoding="utf-8")
    write_mask = source[
        source.index("    def _write_mask("):
        source.index("    def _reload_map(")
    ]
    raster = write_mask.index("rasterize_polygons(")
    reject = write_mask.index("if representation_error:")
    cost = write_mask.index("cost_mask_from_binary(")
    stage = write_mask.index("self.runtime_dir.mkdir(")

    assert raster < reject < cost < stage
    assert "return False, representation_error" in write_mask


def test_unrepresentable_zone_keeps_previous_active_document() -> None:
    old_document = {"type": "FeatureCollection", "features": [{"id": "old"}]}
    new_document = {"type": "FeatureCollection", "features": [{"id": "new"}]}
    reload_calls = []
    manager = SimpleNamespace(
        _require_map_server_active=lambda: (True, ""),
        _document=old_document,
        _document_text="old-json",
        _projected_polygons=[],
        _projected_revision=7,
        _write_mask=lambda _document: (False, "outside=zone_new", None),
        _reload_map=lambda: reload_calls.append(True) or (True, ""),
    )

    result = ZonesManager._apply(manager, new_document, persist=True)

    assert result == (False, "outside=zone_new", 0, 0)
    assert manager._document is old_document
    assert manager._document_text == "old-json"
    assert manager._projected_revision == 7
    assert not reload_calls


def test_projected_keepout_message_filters_disabled_and_preserves_holes() -> None:
    polygons = [
        {
            "id": "disabled",
            "enabled": False,
            "outer_xy": [{"x": 99.0, "y": 99.0}],
            "holes_xy": [],
        },
        {
            "id": "zone_a",
            "enabled": True,
            "outer_xy": [
                {"x": 1.0, "y": 2.0},
                {"x": 3.0, "y": 2.0},
                {"x": 1.0, "y": 2.0},
            ],
            "holes_xy": [[
                {"x": 1.5, "y": 2.1},
                {"x": 2.0, "y": 2.1},
                {"x": 1.5, "y": 2.1},
            ]],
        },
        {
            "id": "zone_a",
            "enabled": True,
            "outer_xy": [
                {"x": 5.0, "y": 6.0},
                {"x": 6.0, "y": 6.0},
                {"x": 5.0, "y": 6.0},
            ],
            "holes_xy": [],
        },
    ]

    message = projected_keepout_state_message(
        "map",
        4,
        Time(sec=12, nanosec=34),
        polygons,
    )

    assert message.header.frame_id == "map"
    assert message.header.stamp.sec == 12
    assert message.header.stamp.nanosec == 34
    assert message.revision == 4
    assert [item.zone_id for item in message.polygons] == ["zone_a", "zone_a"]
    assert len(message.polygons[0].outer.points) == 3
    assert len(message.polygons[0].holes) == 1
    assert len(message.polygons[0].holes[0].points) == 3


def test_accept_active_projected_state_advances_revision_and_publishes() -> None:
    class FakePublisher:
        def __init__(self) -> None:
            self.messages = []

        def publish(self, message) -> None:
            self.messages.append(message)

    class FakeNow:
        def __init__(self, sec: int) -> None:
            self.sec = sec

        def to_msg(self):
            return Time(sec=self.sec)

    class FakeClock:
        def __init__(self) -> None:
            self.sec = 100

        def now(self):
            self.sec += 1
            return FakeNow(self.sec)

    publisher = FakePublisher()
    manager = SimpleNamespace(
        _lock=threading.Lock(),
        _document={"type": "FeatureCollection", "features": []},
        _document_text="",
        _mask_ready=False,
        _mask_source="none",
        _projected_revision=0,
        _projected_polygons=[],
        _projected_pub=publisher,
        map_frame="map",
        get_clock=FakeClock().now,
    )

    ZonesManager._accept_active_state(
        manager,
        {"type": "FeatureCollection", "features": []},
        [],
        mask_source="bootstrap_empty_confirmed",
    )
    ZonesManager._accept_active_state(
        manager,
        {"type": "FeatureCollection", "features": [{"type": "Feature"}]},
        [{
            "id": "zone_b",
            "enabled": True,
            "outer_xy": [{"x": 1.0, "y": 1.0}],
            "holes_xy": [],
        }],
        mask_source="map_server_load_map+global_costmap_clear",
    )

    assert manager._projected_revision == 2
    assert len(publisher.messages) == 2
    assert publisher.messages[0].revision == 1
    assert not publisher.messages[0].polygons
    assert publisher.messages[1].revision == 2
    assert publisher.messages[1].polygons[0].zone_id == "zone_b"
    assert manager._mask_ready is True


def test_initial_empty_state_skips_full_mask_reload_and_optional_clear_wait() -> None:
    source = ZONES_SOURCE.read_text(encoding="utf-8")
    initial = source[
        source.index("    def _load_initial_state("):
        source.index("    def _await(")
    ]
    reload_map = source[
        source.index("    def _reload_map("):
        source.index("    def _apply(")
    ]

    assert 'mask_source="bootstrap_empty_confirmed"' in initial
    assert "self._accept_active_state(" in initial
    assert "if zones_document_is_empty(document):" in initial
    assert "if self._clear_global.service_is_ready():" in reload_map



def test_retiring_startup_watchers_is_idempotent_and_freezes_diagnostics() -> None:
    class FakeTimer:
        def __init__(self) -> None:
            self.cancelled = 0

        def cancel(self) -> None:
            self.cancelled += 1

    class FakeListener:
        def __init__(self) -> None:
            self.unregistered = 0

        def unregister(self) -> None:
            self.unregistered += 1

    timer = FakeTimer()
    listener = FakeListener()
    destroyed = []
    frozen = object()
    coordinator = SimpleNamespace(
        _readiness_retired=False,
        _terminal_snapshot=None,
        _tick_timer=timer,
        _readiness_subscriptions=["clock", "odom"],
        _tf_listener=listener,
        _snapshot=lambda: frozen,
        destroy_subscription=destroyed.append,
    )

    Nav2StartupCoordinator._retire_readiness_watchers(coordinator)
    Nav2StartupCoordinator._retire_readiness_watchers(coordinator)

    assert coordinator._readiness_retired is True
    assert coordinator._terminal_snapshot is frozen
    assert coordinator._readiness_subscriptions == []
    assert destroyed == ["clock", "odom"]
    assert timer.cancelled == 1
    assert listener.unregistered == 1


def test_startup_only_scan_and_keepout_subscriptions_are_conditional() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    assert "if self._obstacle_detection_required:" in source
    assert "if self._use_keepout:" in source
    assert "self._tf_listener.unregister()" in source
    assert "self._terminal_snapshot or self._snapshot()" in source
