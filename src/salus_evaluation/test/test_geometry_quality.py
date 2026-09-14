import math

import pytest

from salus_evaluation.geometry_quality import quality_metrics, resample_polyline, valid_fillet_r4


def test_resampling_and_quality_are_deterministic_and_measure_straight_cleanly():
    points = ((0.0, 0.0), (1.0, 0.0), (3.0, 0.0))
    assert resample_polyline(points, 0.5) == resample_polyline(points, 0.5)
    metrics = quality_metrics(points, points, spacing_m=0.5)
    assert metrics["total_heading_variation_rad"] == pytest.approx(0.0)
    assert metrics["curvature_sign_changes"] == 0
    assert metrics["lateral_error_rms_m"] == pytest.approx(0.0)


def test_quality_reports_wobble_curvature_and_lateral_sign_changes():
    points = ((0.0, 0.0), (1.0, 0.2), (2.0, -0.2), (3.0, 0.2), (4.0, 0.0))
    metrics = quality_metrics(points, ((0.0, 0.0), (4.0, 0.0)), spacing_m=0.25)
    assert metrics["total_heading_variation_rad"] > 0.0
    assert metrics["curvature_sign_changes"] >= 2
    assert metrics["lateral_error_sign_changes"] >= 2
    assert metrics["max_abs_lateral_error_m"] == pytest.approx(0.2, abs=0.02)


def test_valid_fillet_r4_has_tangent_entry_exit_and_deterministic_arc():
    result = valid_fillet_r4((0.0, 0.0), (8.0, 0.0), (8.0, 8.0))
    assert result["valid"]
    assert result["radius_m"] == pytest.approx(4.0)
    assert result["deflection_rad"] == pytest.approx(math.pi / 2.0)
    assert result["tangent_distance_m"] == pytest.approx(4.0)
    assert result["tangent_entry"] == pytest.approx((4.0, 0.0))
    assert result["tangent_exit"] == pytest.approx((8.0, 4.0))
    assert result["arc_points"][0] == pytest.approx(result["tangent_entry"])
    assert result["arc_points"][-1] == pytest.approx(result["tangent_exit"])
    assert result["arc_headings_rad"][0] == pytest.approx(0.0)
    assert result["arc_headings_rad"][-1] == pytest.approx(math.pi / 2.0)


@pytest.mark.parametrize("corner", [
    ((0.0, 0.0), (0.0, 0.0), (1.0, 0.0)),
    ((0.0, 0.0), (1.0, 0.0), (0.0, 0.0)),
    ((0.0, 0.0), (1.0, 0.0), (1.1, 0.0)),
])
def test_valid_fillet_rejects_degenerate_short_or_u_turn_geometry(corner):
    assert not valid_fillet_r4(*corner)["valid"]
