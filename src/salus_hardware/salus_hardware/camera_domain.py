"""Pure camera PTZ models and policies.

This module deliberately contains no ROS, filesystem, clock, or HTTP access.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping


@dataclass(frozen=True)
class CameraLimits:
    pan_min_deg: float = 0.0
    pan_max_deg: float = 355.0
    tilt_min_deg: float = 0.0
    tilt_max_deg: float = 90.0
    zoom_min: float = 1.0
    zoom_max: float = 4.0

    def __post_init__(self) -> None:
        values = tuple(vars(self).values())
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("camera limits must be finite")
        if self.pan_min_deg > self.pan_max_deg or self.tilt_min_deg > self.tilt_max_deg:
            raise ValueError("camera angular limits must be ordered")
        if self.zoom_min > self.zoom_max:
            raise ValueError("camera zoom limits must be ordered")


@dataclass(frozen=True)
class PtzPose:
    pan_deg: float
    tilt_deg: float
    zoom_level: float


@dataclass(frozen=True)
class PresetDefinition:
    name: str
    pose: PtzPose
    editable: bool
    saves_zoom: bool


@dataclass(frozen=True)
class CameraState:
    available: bool
    pose: PtzPose | None
    zoom_in: bool
    last_command: str
    active_preset: str
    error: str = ""


@dataclass(frozen=True)
class CameraCommandResult:
    ok: bool
    error: str
    state: CameraState


PRESET_ALIASES = {"center": "home", "back": "rear"}
SAVEABLE_PRESETS = frozenset({"home", "left", "right"})


def normalize_pan(angle_deg: float, limits: CameraLimits) -> float:
    _finite(angle_deg, "pan_deg")
    normalized = float(angle_deg) % 360.0
    return clamp(normalized, limits.pan_min_deg, limits.pan_max_deg)


def clamp(value: float, lower: float, upper: float) -> float:
    _finite(value, "camera value")
    return max(lower, min(upper, float(value)))


def normalize_pose(pose: PtzPose, limits: CameraLimits) -> PtzPose:
    return PtzPose(
        pan_deg=normalize_pan(pose.pan_deg, limits),
        tilt_deg=clamp(pose.tilt_deg, limits.tilt_min_deg, limits.tilt_max_deg),
        zoom_level=clamp(pose.zoom_level, limits.zoom_min, limits.zoom_max),
    )


def resolve_preset(name: str, presets: Mapping[str, PresetDefinition]) -> str:
    normalized = str(name or "").strip().lower()
    if not normalized:
        raise ValueError("preset is required")
    canonical = PRESET_ALIASES.get(normalized, normalized)
    if canonical not in presets:
        raise ValueError(f"unsupported preset '{name}'")
    return canonical


def target_pose(
    current: PtzPose,
    limits: CameraLimits,
    *,
    relative: bool,
    apply_pan: bool,
    pan_deg: float,
    apply_tilt: bool,
    tilt_deg: float,
    apply_zoom: bool,
    zoom_level: float,
) -> PtzPose:
    """Resolve a selected-axis command to one bounded absolute pose."""
    values = (pan_deg, tilt_deg, zoom_level)
    if any(not math.isfinite(float(value)) for value in values):
        raise ValueError("camera command values must be finite")
    pan = current.pan_deg
    tilt = current.tilt_deg
    zoom = current.zoom_level
    if apply_pan:
        pan = current.pan_deg + pan_deg if relative else pan_deg
    if apply_tilt:
        tilt = current.tilt_deg + tilt_deg if relative else tilt_deg
    if apply_zoom:
        zoom = current.zoom_level + zoom_level if relative else zoom_level
    return normalize_pose(PtzPose(pan, tilt, zoom), limits)


def circular_pan_error(a_deg: float, b_deg: float) -> float:
    return abs(((float(a_deg) - float(b_deg) + 180.0) % 360.0) - 180.0)


def matching_preset(
    pose: PtzPose,
    presets: Mapping[str, PresetDefinition],
    *,
    angle_tolerance_deg: float = 1.5,
    zoom_tolerance: float = 0.2,
) -> str:
    for name, preset in presets.items():
        target = preset.pose
        if (
            circular_pan_error(pose.pan_deg, target.pan_deg) <= angle_tolerance_deg
            and abs(pose.tilt_deg - target.tilt_deg) <= angle_tolerance_deg
            and abs(pose.zoom_level - target.zoom_level) <= zoom_tolerance
        ):
            return name
    return ""


def saved_preset(
    name: str,
    current: PtzPose,
    presets: Mapping[str, PresetDefinition],
    limits: CameraLimits,
    *,
    save_zoom: bool,
) -> PresetDefinition:
    canonical = resolve_preset(name, presets)
    preset = presets[canonical]
    if canonical not in SAVEABLE_PRESETS or not preset.editable:
        raise ValueError(f"preset '{canonical}' cannot be overwritten from the UI")
    # The service field is part of the legacy contract. The canonical policy
    # remains authoritative: HOME stores zoom, lateral presets preserve it.
    effective_save_zoom = preset.saves_zoom
    if canonical == "home" and not save_zoom:
        raise ValueError("home preset must save zoom")
    if canonical in {"left", "right"} and save_zoom:
        raise ValueError(f"preset '{canonical}' must preserve its configured zoom")
    normalized = normalize_pose(current, limits)
    zoom = normalized.zoom_level if effective_save_zoom else preset.pose.zoom_level
    return PresetDefinition(canonical, PtzPose(normalized.pan_deg, normalized.tilt_deg, zoom), True, effective_save_zoom)


def default_presets(limits: CameraLimits, *, home_zoom: float = 1.0) -> dict[str, PresetDefinition]:
    def make(name: str, pan: float, editable: bool, saves_zoom: bool) -> PresetDefinition:
        return PresetDefinition(name, normalize_pose(PtzPose(pan, 0.0, home_zoom), limits), editable, saves_zoom)
    return {
        "home": make("home", 0.0, True, True),
        "front": make("front", 0.0, False, False),
        "left": make("left", 90.0, True, False),
        "right": make("right", 270.0, True, False),
        "rear": make("rear", 180.0, False, False),
    }


def _finite(value: float, name: str) -> None:
    if not math.isfinite(float(value)):
        raise ValueError(f"{name} must be finite")
