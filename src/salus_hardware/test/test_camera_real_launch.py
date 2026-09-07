"""Structural tests for the real PTZ owner."""

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from launch_ros.actions import Node


LAUNCH = Path(__file__).parents[1] / "launch" / "camera_real.launch.py"


def _module():
    spec = spec_from_file_location("camera_real", LAUNCH)
    module = module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_real_camera_has_one_isapi_owner_and_no_video_owner() -> None:
    source = LAUNCH.read_text(encoding="utf-8")
    description = _module().generate_launch_description()
    nodes = [entity for entity in description.entities if isinstance(entity, Node)]

    assert len(nodes) == 1
    assert 'executable="camera_node"' in source
    assert 'name="salus_camera"' in source
    assert '"backend": "isapi"' in source
    assert '"use_sim_time": False' in source
    assert "mediamtx" not in source.lower()
    assert "image_raw" not in source


def test_real_camera_uses_runtime_preset_path_and_external_secret() -> None:
    source = LAUNCH.read_text(encoding="utf-8")

    assert "/ros2_ws/log/runtime/camera/presets.json" in source
    assert "CAMERA_PASS" not in source
    assert "camera_pass" not in source.lower()
