import math
from salus_perception.scan_filters import clean_ranges, is_ground_point, obstacle_points

def test_ground_filter_keeps_curb_and_removes_flat_ground() -> None:
    points=[(1.0,0.0,0.0),(1.0,0.0,0.15),(1.0,0.0,0.35)]
    assert is_ground_point(*points[0]) and is_ground_point(*points[1])
    assert obstacle_points(points)==[points[2]]


def test_ground_filter_rejects_invalid_and_distant_points() -> None:
    assert not is_ground_point(float("nan"), 0.0, 0.0)
    assert not is_ground_point(25.0, 0.0, 0.0)

def test_scan_noise_filter_rejects_invalid_and_isolated_speckles() -> None:
    cleaned=clean_ranges([math.nan,0.1,2.0,10.0,10.1,10.0,30.0])
    assert math.isinf(cleaned[0]) and math.isinf(cleaned[1]) and math.isinf(cleaned[2])
    assert cleaned[3:6]==[10.0,10.1,10.0] and math.isinf(cleaned[6])
