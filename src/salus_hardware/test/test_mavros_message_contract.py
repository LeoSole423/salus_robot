from mavros_msgs.msg import GPSRAW, RTCM


def test_mavros_message_types_and_rtk_fix_constants_are_available() -> None:
    assert GPSRAW is not None
    assert RTCM is not None
    assert GPSRAW.GPS_FIX_TYPE_RTK_FLOAT == 5
    assert GPSRAW.GPS_FIX_TYPE_RTK_FIXED == 6
