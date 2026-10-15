from dataclasses import replace
import json
import math
import struct

import pytest

from salus_hardware.bmi088_domain import (
    Calibration, FrameParser, SequenceTracker, crc8, decode, estimate_time, request, sample_stamp,
)
from salus_hardware.imu_comparison_domain import metrics, rotate


def packet(token=0x123456, now=1001000, received=1000000, sequence=1, boot=42, flags=7):
    f = bytearray(128)
    f[:8] = bytes([0x5A, 0x10, 3, 128, 1, flags, 30, 15])
    struct.pack_into('<I', f, 12, 1)
    struct.pack_into('<II', f, 28, sequence, sequence)
    struct.pack_into('<QQQ', f, 36, now-2000, now-1000, now-10000)
    struct.pack_into('<hhh', f, 60, 0, 0, 5460)
    struct.pack_into('<hhh', f, 66, 10, -20, 30)
    struct.pack_into('<fff', f, 72, 0., 0., 9.80665)
    struct.pack_into('<fff', f, 84, 0.01, -0.02, 0.03)
    struct.pack_into('<iQ', f, 96, 25000, now)
    f[108] = 1
    f[111:114] = token.to_bytes(3, 'little')
    struct.pack_into('<IQ', f, 114, boot, received)
    f[127] = crc8(f[:-1])
    return bytes(f)


def profile():
    return {'schema_version': 1, 'kind': 'salus_bmi088_calibration', 'device_id': 'board-1',
            'profile_id': 'profile-1', 'frame': 'bosch_sensor', 'chip_ids': [30, 15],
            'acquisition_profile': 1,
            'gyro': {'method': 'stationary_mean', 'bias_rad_s': [0.01, -0.02, 0.03],
                     'variance_rad2_s2': [1e-6]*3, 'temperature_c': 25},
            'accel': {'method': 'six_position_diagonal', 'offset_m_s2': [0., 0., 0.1],
                      'scale': [1., 1., 1.01], 'variance_m2_s4': [0.001]*3,
                      'temperature_c': 25}}


def test_codec_fragmentation_corruption_and_bounded_noise():
    good = packet()
    broken = bytearray(good)
    broken[-1] ^= 1
    stream = b'noise' + bytes(broken) + good + good
    parser = FrameParser()
    out = []
    for i in range(0, len(stream), 3):
        out.extend(parser.feed(stream[i:i+3]))
        assert len(parser.buffer) < 128
    assert out == [good, good]
    assert decode(good).token == 0x123456
    assert decode(good).gyro.values == pytest.approx((0.01, -0.02, 0.03))
    assert request(0x123456)[:6] == bytes([0xAB, 0x10, 3, 0x56, 0x34, 0x12])
    assert request(1)[6] == crc8(request(1)[:6])
    parser.feed(b'x' * 100000)
    assert len(parser.buffer) < 128


def test_invalid_payloads_and_token_range():
    for offset, value in ((6, 0), (108, 2), (109, 1), (126, 1), (5, 128)):
        data = bytearray(packet())
        data[offset] = value
        data[-1] = crc8(data[:-1])
        with pytest.raises(ValueError):
            decode(bytes(data))
    data = bytearray(packet())
    struct.pack_into('<f', data, 84, float('nan'))
    data[-1] = crc8(data[:-1])
    with pytest.raises(ValueError):
        decode(bytes(data))
    for data in (packet()[:-1], packet()[:-1] + b'\xFF'):
        with pytest.raises(ValueError):
            decode(data)
    with pytest.raises(ValueError):
        request(0x1000000)


def test_four_timestamps_and_sensor_times_are_independent():
    sample = decode(packet())
    timing = estimate_time(sample, 10000000000, 10003000000, 1000000000000, 20000000)
    assert timing.uncertainty_ns == 1000000
    gyro = sample_stamp(sample.gyro, sample, timing, 1000000000000, 100000000)
    accel = sample_stamp(sample.accel, sample, timing, 1000000000000, 100000000)
    assert gyro == 999998000000
    assert accel == gyro - 1000000
    delayed = sample_stamp(sample.gyro, sample, timing, 1000000000000, 100000000, 7000000)
    assert delayed == gyro-7000000
    for sent, received, ros in ((10, 9, 100), (0, 30000000, 100), (0, 1, 0)):
        with pytest.raises(ValueError):
            estimate_time(sample, sent, received, ros, 20000000)
    with pytest.raises(ValueError):
        estimate_time(replace(sample, received_us=sample.capture_us+1), 0, 3000000, 10, 20000000)
    for vector in (replace(sample.gyro, valid=False), replace(sample.gyro, saturated=True),
                   replace(sample.gyro, timestamp_us=sample.capture_us+1),
                   replace(sample.gyro, timestamp_us=1)):
        with pytest.raises(ValueError):
            sample_stamp(vector, sample, timing, 1000000000000, 100000000)


def test_sequence_reset_and_nonmonotonic_ros_rejected():
    tracker = SequenceTracker()
    sample = decode(packet())
    assert tracker.accept(sample, 'gyro', 100)
    assert not tracker.accept(sample, 'gyro', 101)
    assert tracker.accept(decode(packet(sequence=2, now=1002000)), 'gyro', 102)
    assert not tracker.accept(decode(packet(sequence=3, now=1003000)), 'gyro', 101)
    reboot = decode(packet(sequence=1, now=20000, received=19000, boot=43))
    assert tracker.accept(reboot, 'gyro', 103)


def test_calibration_schema_identity_and_correction(tmp_path):
    d = profile()
    path = tmp_path/'profile.json'
    path.write_text(json.dumps(d))
    calibration = Calibration.load(str(path), 'board-1')
    assert calibration.gyro.apply((0.01, -0.02, 0.03)) == pytest.approx((0., 0., 0.))
    assert calibration.accel.apply((0., 0., 9.80665)) == pytest.approx((0., 0., 9.8037165))
    with pytest.raises(ValueError):
        Calibration.parse(d, 'wrong-board')
    d['gyro']['bias_rad_s'][0] = float('inf')
    with pytest.raises(ValueError):
        Calibration.parse(d, 'board-1')
    path.write_text('{"schema_version":1,"schema_version":2}')
    with pytest.raises(ValueError):
        Calibration.load(str(path), 'board-1')


def test_common_frame_rotation_and_statistics():
    rotated = rotate((1., 0., 0.), (0., 0., math.sqrt(0.5), math.sqrt(0.5)))
    assert rotated == pytest.approx((0., 1., 0.))
    assert metrics([])['mean_error_rad_s'] is None
    result = metrics([(1., 2., 0.), (-1., 2., 0.)])
    assert result['mean_error_rad_s'] == [0., 2., 0.]
    assert result['rms_error_rad_s'] == [1., 2., 0.]
