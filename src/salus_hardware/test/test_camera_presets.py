from pathlib import Path

from salus_hardware.camera_domain import CameraLimits, PtzPose, default_presets, saved_preset
from salus_hardware.camera_presets import PresetRepository


def test_corrupt_preset_file_keeps_defaults(tmp_path: Path) -> None:
    path = tmp_path / "presets.json"
    path.write_text("{not-json", encoding="utf-8")
    defaults = default_presets(CameraLimits())
    assert PresetRepository(path, CameraLimits()).load(defaults) == defaults


def test_saved_presets_survive_repository_reload(tmp_path: Path) -> None:
    limits = CameraLimits()
    presets = default_presets(limits)
    presets["home"] = saved_preset("home", PtzPose(34.0, 12.0, 3.0), presets, limits, save_zoom=True)
    repository = PresetRepository(tmp_path / "presets.json", limits)
    repository.save(presets)
    assert repository.load(default_presets(limits))["home"].pose == PtzPose(34.0, 12.0, 3.0)
