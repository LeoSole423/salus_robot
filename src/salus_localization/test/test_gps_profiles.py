from sensor_msgs.msg import NavSatFix
from salus_localization.gps_profiles import SimGpsFixProcessor, resolve_gps_profile


def make_fix() -> NavSatFix:
    message = NavSatFix(); message.header.stamp.sec = 10; message.latitude = -31.4858037; message.longitude = -64.2410570
    return message


def test_profiles_have_stable_names() -> None:
    assert tuple(sorted(resolve_gps_profile(name).name for name in ("ideal", "f9p_rtk", "m8n"))) == ("f9p_rtk", "ideal", "m8n")


def test_ideal_profile_preserves_position() -> None:
    source = make_fix(); output = SimGpsFixProcessor(resolve_gps_profile("ideal"), 123).process(source)
    assert output is not None and output.latitude == source.latitude and output.longitude == source.longitude


def test_m8n_profile_throttles_samples() -> None:
    processor = SimGpsFixProcessor(resolve_gps_profile("m8n"), 123)
    assert processor.process(make_fix()) is not None
    second = make_fix(); second.header.stamp.nanosec = 50_000_000
    assert processor.process(second) is None
