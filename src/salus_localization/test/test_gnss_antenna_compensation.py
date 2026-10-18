import math
import pytest
from salus_localization.gnss_antenna_compensation import (
    MapHeading, MapHeadingHistory, compensate_antenna, rotate_covariance)


def test_antenna_mount_rotates_to_base_and_propagates_yaw_uncertainty():
    x,y,cov = compensate_antenna(10.66,20.2,MapHeading(1,0,0.1),0.66,0.2,(0.05,0,0.05))
    assert x == pytest.approx(10)
    assert y == pytest.approx(20)
    assert cov == pytest.approx((0.054,-0.0132,0.09356))
    x,y,_ = compensate_antenna(9.8,20.66,MapHeading(1,math.pi/2,0),0.66,0.2,(0.05,0,0.05))
    assert (x,y) == pytest.approx((10,20))


def test_map_heading_handles_arbitrary_local_origin_and_alternate_mount():
    # Only the map heading is an input; an unrelated odom yaw cannot rotate this offset.
    x,y,_ = compensate_antenna(9,20,MapHeading(1,math.pi,0),1,0,(0.1,0,0.1))
    assert (x,y) == pytest.approx((10,20))
    assert rotate_covariance(0.04,0,0.09,-math.pi/2) == pytest.approx((0.09,0,0.04))


def test_history_wraps_yaw_and_never_extrapolates_across_missing_data():
    history = MapHeadingHistory()
    history.add(MapHeading(1,math.pi-0.02,0.01))
    history.add(MapHeading(1.1,-math.pi+0.02,0.04))
    result = history.at(1.05)
    assert abs(result.yaw_rad) == pytest.approx(math.pi)
    assert result.variance_rad2 == 0.04
    assert history.at(0.9) is None
    assert history.at(1.2) is None
    history.add(MapHeading(2,0,0.01))
    assert history.at(1.5) is None
    assert not history.add(MapHeading(1.9,0,0.1))
    assert not history.add(MapHeading(2.1,0,-0.1))


def test_invalid_geometry_or_covariance_does_not_create_a_measurement():
    with pytest.raises(ValueError):
        compensate_antenna(0,0,MapHeading(1,0,0.1),math.nan,0,(0.1,0,0.1))
    with pytest.raises(ValueError):
        rotate_covariance(0.01,0.2,0.01,0)


def test_diagnostic_no_compensation_preserves_covariance_and_antenna_position():
    args = (10.66,20.2,MapHeading(1,0,0.1),0.66,0.2,(0.05,0,0.05))
    corrected = compensate_antenna(*args)
    diagnostic = compensate_antenna(*args,compensate_position=False)
    assert diagnostic[:2] == pytest.approx((10.66,20.2))
    assert diagnostic[2] == corrected[2]
