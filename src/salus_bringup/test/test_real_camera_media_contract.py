"""Composition contract for real camera/PTZ and external MediaMTX."""

from pathlib import Path


ROOT = Path(__file__).parents[3]
MVP = ROOT / "src/salus_bringup/launch/real_mvp.launch.py"
CAMERA = ROOT / "src/salus_hardware/launch/camera_real.launch.py"
INTENT = ROOT / "docs/migration-evidence/intent/camera-ptz-real-runtime-20260907.md"


def test_real_mvp_contains_exactly_one_camera_owner_and_no_media_owner() -> None:
    source = MVP.read_text(encoding="utf-8")

    assert source.count('"camera_real.launch.py"') == 1
    assert source.count('"salus_hardware",\n            "camera_real.launch.py"') == 1
    assert "mediamtx" not in source.lower()
    assert "camera_real.launch.py" in source


def test_real_camera_contract_keeps_media_external_and_optional() -> None:
    camera_source = CAMERA.read_text(encoding="utf-8")
    intent = INTENT.read_text(encoding="utf-8")

    assert '"use_sim_time": False' in camera_source
    assert "MediaMTX sigue siendo un servicio del host" in intent
    assert "cam3" in intent
    assert "8889" in intent
    assert "on-demand" in intent
    assert "readiness/authority" in intent
    assert "camera" in intent.lower()
