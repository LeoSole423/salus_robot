"""Atomic persistence for locally calibrated camera presets."""

from __future__ import annotations

from dataclasses import replace
import json
import os
from pathlib import Path
from typing import Mapping

from .camera_domain import CameraLimits, PresetDefinition, PtzPose, normalize_pose


class PresetRepository:
    def __init__(self, path: Path, limits: CameraLimits) -> None:
        self._path = path
        self._limits = limits

    def load(self, base: Mapping[str, PresetDefinition]) -> dict[str, PresetDefinition]:
        presets = dict(base)
        if not self._path.exists():
            return presets
        try:
            raw = json.loads(self._path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return presets
        if not isinstance(raw, dict):
            return presets
        for name, value in raw.items():
            if name not in presets or not isinstance(value, dict):
                continue
            try:
                pose = normalize_pose(PtzPose(
                    float(value["pan_deg"]), float(value["tilt_deg"]), float(value["zoom_level"])
                ), self._limits)
            except (KeyError, TypeError, ValueError):
                continue
            presets[name] = replace(presets[name], pose=pose)
        return presets

    def save(self, presets: Mapping[str, PresetDefinition]) -> None:
        document = {
            name: {"pan_deg": item.pose.pan_deg, "tilt_deg": item.pose.tilt_deg, "zoom_level": item.pose.zoom_level}
            for name, item in sorted(presets.items()) if item.editable
        }
        self._path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._path.with_suffix(self._path.suffix + ".tmp")
        try:
            with temporary.open("w", encoding="utf-8") as handle:
                json.dump(document, handle, indent=2, sort_keys=True)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, self._path)
        except OSError:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            raise
