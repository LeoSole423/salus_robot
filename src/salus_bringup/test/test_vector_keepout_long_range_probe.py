import importlib.util
from pathlib import Path
import sys

PROBE = Path(__file__).parents[3] / "tools" / "smoke_navigation_vector_keepout_long_range.py"
sys.path.insert(0, str(PROBE.parent))
SPEC = importlib.util.spec_from_file_location("vector_keepout_long_range", PROBE)
probe = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = probe
SPEC.loader.exec_module(probe)


def test_map_point_to_odom_identity_translation_and_rotation():
    assert probe.map_point_to_odom(2.0, -3.0, (0.0, 0.0, 0.0)) == (2.0, -3.0)
    assert probe.map_point_to_odom(2.0, -3.0, (5.0, 4.0, 0.0)) == (7.0, 1.0)
    x, y = probe.map_point_to_odom(2.0, 0.0, (0.0, 0.0, 1.5707963267948966))
    assert abs(x) < 1e-12 and abs(y - 2.0) < 1e-12
    x, y = probe.map_point_to_odom(2.0, 3.0, (1.0, 2.0, 1.5707963267948966))
    assert abs(x + 2.0) < 1e-12 and abs(y - 4.0) < 1e-12


def test_costmap_indices_floor_negative_coordinates():
    assert probe.math.floor((-0.01 - 0.0) / 0.1) == -1


def test_explicit_detour_distance_is_relative_to_the_requested_segment():
    start, goal = (341.0, 0.0), (375.0, 0.0)
    assert probe.perpendicular_distance((355.0, 0.0), start, goal) == 0.0
    assert probe.perpendicular_distance((355.0, 2.5), start, goal) == 2.5


def test_long_range_probe_uses_an_explicit_compute_path_start():
    source = PROBE.read_text(encoding="utf-8")
    assert "request.use_start=True" in source
    assert "request.start=pose(*start)" in source
