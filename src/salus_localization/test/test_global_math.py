import math
from sensor_msgs.msg import NavSatFix
from salus_localization.gps_course_heading import CourseHeadingEstimator, heading_from_fixes
from salus_localization.global_stationary_gates import is_stationary
from salus_localization.map_gps_absolute_measurement import project_fix


def test_project_fix_uses_enu_axes() -> None:
    x_m, y_m = project_fix(-31.4858037 + 1.0 / 111_320.0, -64.2410570, -31.4858037, -64.2410570)
    assert abs(x_m) < 0.01 and math.isclose(y_m, 1.0, abs_tol=0.01)


def test_project_fix_rotates_into_the_datum_aligned_map_frame() -> None:
    x_m, y_m = project_fix(0.0, 0.0001, 0.0, 0.0, datum_yaw_deg=90.0)
    assert math.isclose(x_m, 0.0, abs_tol=0.01)
    assert math.isclose(y_m, -11.132, abs_tol=0.02)


def test_gps_heading_is_east_for_increasing_longitude() -> None:
    first = NavSatFix(); first.latitude = -31.48; first.longitude = -64.24
    second = NavSatFix(); second.latitude = first.latitude; second.longitude = first.longitude + 10.0 / (111_320.0 * math.cos(math.radians(first.latitude)))
    yaw, distance = heading_from_fixes(first, second)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6) and distance > 9.9


def test_course_heading_never_relabels_previous_heading_during_turn() -> None:
    estimator = CourseHeadingEstimator()
    for i in range(41):
        t = 1.0+i*0.05
        estimator.update_motion(stamp_s=t, now_s=t, speed_mps=3.0,
            steer_deg=0.0 if t <= 2 else 12.0, valid=True,
            yaw_rate_rps=0.0 if t <= 2 else 0.3)
        estimator.add_fix(-31.0, -64.0 + (t-1)*3/(111_320*math.cos(math.radians(-31))), t)
        result = estimator.estimate(now_s=t, speed_mps=3.0, steer_deg=12.0,
            steer_valid=True, yaw_rate_rps=0.3)
        assert not result.reason.startswith('hold_')
    assert estimator.turn.state == 'STRONG_TURN'
    assert not result.valid


def test_course_heading_rejects_stale_fix_and_stationary_gate_requires_fresh_telemetry() -> None:
    estimator = CourseHeadingEstimator()
    estimator.add_fix(-31.0, -64.0, 1.0)
    estimator.update_motion(stamp_s=2.0, now_s=2.0, speed_mps=1.0, steer_deg=0.0, valid=True, yaw_rate_rps=0.0)
    assert estimator.estimate(now_s=2.0, speed_mps=1.0, steer_deg=0.0, steer_valid=True, yaw_rate_rps=0.0).reason == "stale_fix"
    assert is_stationary(0.02, True, True) and not is_stationary(0.02, False, True)


def test_post_turn_heading_uses_only_post_exit_fixes():
    estimator = CourseHeadingEstimator(min_distance_m=0.5)
    for i in range(101):
        t = round(i*0.05, 8)
        rate = 0.3 if 1 <= t < 2 else 0
        estimator.update_motion(stamp_s=t, now_s=t, speed_mps=1.2,
            steer_deg=12 if rate else 1, valid=True, yaw_rate_rps=rate)
        # Deliberately incompatible pre/post-turn GNSS chords.
        lat = -31 + max(0, t-2)*1.2/111320
        lon = -64 + min(t, 2)*1.2/(111320*math.cos(math.radians(-31)))
        estimator.add_fix(lat, lon, t)
        result = estimator.estimate(now_s=t, speed_mps=1.2, steer_deg=1,
            steer_valid=True, yaw_rate_rps=rate)
        if t > 3 and result.valid:
            assert math.isclose(result.yaw_rad, math.pi/2, abs_tol=1e-5)
            assert estimator.fixes[0][2] > 2
    assert result.valid
