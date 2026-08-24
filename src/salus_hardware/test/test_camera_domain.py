import pytest

from salus_hardware.camera_domain import (
    CameraLimits, PtzPose, default_presets, matching_preset, normalize_pan,
    saved_preset, target_pose,
)


def test_pan_wraps_then_respects_hardware_limit() -> None:
    limits = CameraLimits()
    assert normalize_pan(-10.0, limits) == 350.0
    assert normalize_pan(359.0, limits) == 355.0


def test_relative_target_keeps_unselected_axes() -> None:
    target = target_pose(
        PtzPose(350.0, 20.0, 2.0), CameraLimits(), relative=True,
        apply_pan=True, pan_deg=20.0, apply_tilt=False, tilt_deg=0.0,
        apply_zoom=False, zoom_level=0.0,
    )
    assert target == PtzPose(10.0, 20.0, 2.0)


def test_preset_alias_and_circular_match() -> None:
    presets = default_presets(CameraLimits())
    assert matching_preset(PtzPose(359.0, 0.0, 1.0), presets) == "home"


def test_save_policy_preserves_lateral_zoom() -> None:
    limits = CameraLimits()
    presets = default_presets(limits)
    left = saved_preset("left", PtzPose(120.0, 12.0, 3.0), presets, limits, save_zoom=False)
    assert left.pose == PtzPose(120.0, 12.0, 1.0)
    home = saved_preset("home", PtzPose(20.0, 5.0, 3.0), presets, limits, save_zoom=True)
    assert home.pose.zoom_level == 3.0
    with pytest.raises(ValueError, match="cannot be overwritten"):
        saved_preset("rear", PtzPose(0.0, 0.0, 1.0), presets, limits, save_zoom=False)
