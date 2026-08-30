from pathlib import Path
from types import SimpleNamespace

from salus_navigation.nav2_startup_coordinator import (
    Nav2StartupCoordinator,
    zones_state_mask_ready,
)
from salus_navigation.zones_manager import zones_document_is_empty


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

    assert 'self._mask_source = "bootstrap_empty_confirmed"' in initial
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
