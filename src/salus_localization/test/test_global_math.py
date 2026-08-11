import math
from sensor_msgs.msg import NavSatFix
from salus_localization.gps_course_heading import heading_from_fixes
from salus_localization.map_gps_absolute_measurement import project_fix


def test_project_fix_uses_enu_axes() -> None:
    x_m, y_m = project_fix(-31.4858037 + 1.0 / 111_320.0, -64.2410570, -31.4858037, -64.2410570)
    assert abs(x_m) < 0.01 and math.isclose(y_m, 1.0, abs_tol=0.01)


def test_gps_heading_is_east_for_increasing_longitude() -> None:
    first = NavSatFix(); first.latitude = -31.48; first.longitude = -64.24
    second = NavSatFix(); second.latitude = first.latitude; second.longitude = first.longitude + 10.0 / (111_320.0 * math.cos(math.radians(first.latitude)))
    yaw, distance = heading_from_fixes(first, second)
    assert math.isclose(yaw, 0.0, abs_tol=1e-6) and distance > 9.9
