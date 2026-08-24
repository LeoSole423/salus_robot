import math

from sensor_msgs.msg import LaserScan

from salus_perception.scan_preview import reduce_scan_preview


def _scan() -> LaserScan:
    message = LaserScan()
    message.header.frame_id = "base_footprint"
    message.angle_min = -1.0
    message.angle_max = 1.0
    message.angle_increment = 0.5
    message.time_increment = 0.01
    message.scan_time = 0.1
    message.range_min = 0.4
    message.range_max = 20.0
    message.ranges = [1.0, 2.0, 13.0, 4.0, 5.0]
    message.intensities = [10.0, 20.0, 30.0, 40.0, 50.0]
    return message


def test_preview_crops_downsamples_and_bounds_ranges_without_intensities() -> None:
    preview = reduce_scan_preview(
        _scan(), beam_stride=2, crop_angle_min_rad=-1.0,
        crop_angle_max_rad=1.0, output_range_max_m=12.0,
    )
    assert preview is not None
    assert preview.header.frame_id == "base_footprint"
    assert math.isclose(preview.angle_min, -1.0)
    assert math.isclose(preview.angle_max, 1.0)
    assert math.isclose(preview.angle_increment, 1.0)
    assert math.isclose(preview.time_increment, 0.02)
    assert list(preview.ranges[:1]) == [1.0]
    assert math.isinf(preview.ranges[1])
    assert list(preview.ranges[2:]) == [5.0]
    assert list(preview.intensities) == []


def test_preview_rejects_empty_or_invalid_scans_without_inventing_output() -> None:
    message = _scan()
    message.ranges = []
    assert reduce_scan_preview(message, beam_stride=1, crop_angle_min_rad=-1.0, crop_angle_max_rad=1.0, output_range_max_m=12.0) is None
    message = _scan()
    message.angle_increment = 0.0
    assert reduce_scan_preview(message, beam_stride=1, crop_angle_min_rad=-1.0, crop_angle_max_rad=1.0, output_range_max_m=12.0) is None
    message = _scan()
    assert reduce_scan_preview(message, beam_stride=1, crop_angle_min_rad=2.0, crop_angle_max_rad=3.0, output_range_max_m=12.0) is None
