from pathlib import Path
from types import SimpleNamespace

from salus_navigation.nav2_startup_coordinator import Nav2StartupCoordinator, zones_state_mask_ready
from salus_navigation.zones_manager import ZonesManager, zones_document_is_empty


ROOT = Path(__file__).parents[1]
SOURCE = ROOT / "salus_navigation" / "nav2_startup_coordinator.py"
ZONES_SOURCE = ROOT / "salus_navigation" / "zones_manager.py"


def test_zones_state_is_required_to_confirm_accepted_vector_state() -> None:
    assert not zones_state_mask_ready(None)
    assert not zones_state_mask_ready(SimpleNamespace(ok=False, mask_ready=True))
    assert not zones_state_mask_ready(SimpleNamespace(ok=True, mask_ready=False))
    assert zones_state_mask_ready(SimpleNamespace(ok=True, mask_ready=True))


def test_startup_uses_zone_authority_without_legacy_mask_subscription() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    assert "def _on_mask" not in source
    assert '"/keepout_filter_mask"' not in source
    assert 'GetZonesState, "/zones_manager/get_state"' in source
    assert "self._poll_zones_state(now)" in source


def test_empty_document_is_a_valid_vector_state() -> None:
    assert zones_document_is_empty({"type": "FeatureCollection", "features": []})
    assert not zones_document_is_empty({"type": "FeatureCollection", "features": [{"id": "zone"}]})


def test_projection_failure_keeps_previous_active_document() -> None:
    old_document = {"type": "FeatureCollection", "features": [{"id": "old"}]}
    manager = SimpleNamespace(_document=old_document, _document_text="old-json", use_keepout=True, _project=lambda _document: (None, "projection failed"))
    result = ZonesManager._apply(manager, {"type": "FeatureCollection", "features": []}, persist=True)
    assert result == (False, "projection failed", 0, 0)
    assert manager._document is old_document
    assert manager._document_text == "old-json"


def test_initial_state_and_apply_have_no_fixed_mask_dependency() -> None:
    source = ZONES_SOURCE.read_text(encoding="utf-8")
    assert "LoadMap" not in source
    assert "keepout_mask.pgm" not in source
    assert "self._apply(" in source[source.index("    def _load_initial_state("):source.index("    def _await(")]


def test_retiring_startup_watchers_is_idempotent_and_freezes_diagnostics() -> None:
    class Timer:
        def __init__(self): self.cancelled = 0
        def cancel(self): self.cancelled += 1
    class Listener:
        def __init__(self): self.unregistered = 0
        def unregister(self): self.unregistered += 1
    timer, listener, destroyed, frozen = Timer(), Listener(), [], object()
    coordinator = SimpleNamespace(_readiness_retired=False, _terminal_snapshot=None, _tick_timer=timer, _readiness_subscriptions=["clock"], _tf_listener=listener, _snapshot=lambda: frozen, destroy_subscription=destroyed.append)
    Nav2StartupCoordinator._retire_readiness_watchers(coordinator)
    Nav2StartupCoordinator._retire_readiness_watchers(coordinator)
    assert coordinator._readiness_retired and coordinator._terminal_snapshot is frozen
    assert destroyed == ["clock"] and timer.cancelled == 1 and listener.unregistered == 1
