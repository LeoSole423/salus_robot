from pathlib import Path
from types import SimpleNamespace

from salus_navigation.nav2_startup_coordinator import zones_state_mask_ready
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
