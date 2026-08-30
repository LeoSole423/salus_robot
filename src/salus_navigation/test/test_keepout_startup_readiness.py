from pathlib import Path
from types import SimpleNamespace

from salus_navigation.nav2_startup_coordinator import zones_state_mask_ready


ROOT = Path(__file__).parents[1]
SOURCE = ROOT / "salus_navigation" / "nav2_startup_coordinator.py"


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
