"""Pure BMI088 diagnostic codec, calibration-v1 reader and time conversion."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import struct
from pathlib import Path


def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ (0x31 if crc & 0x80 else 0)) & 255
    return crc


def request(token: int) -> bytes:
    if not 0 <= token <= 0xFFFFFF:
        raise ValueError('token outside uint24')
    data = bytes([0xAB, 0x10, 3]) + token.to_bytes(3, 'little')
    return data + bytes([crc8(data)])


class FrameParser:
    """Bounded streaming parser; never interpret payload as another frame."""

    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> list[bytes]:
        frames = []
        for byte in data:
            self.buffer.append(byte)
            while self.buffer:
                if self.buffer[0] == 0x55:
                    size = 8
                elif self.buffer[0] == 0x5A:
                    if len(self.buffer) < 4:
                        break
                    size = {bytes([0x10, 1, 24]): 24,
                            bytes([0x10, 2, 112]): 112,
                            bytes([0x10, 3, 128]): 128}.get(bytes(self.buffer[1:4]))
                    if size is None:
                        del self.buffer[0]
                        self.errors += 1
                        continue
                else:
                    del self.buffer[0]
                    self.errors += 1
                    continue
                if len(self.buffer) < size:
                    break
                frame = bytes(self.buffer[:size])
                if crc8(frame[:-1]) != frame[-1]:
                    del self.buffer[0]
                    self.errors += 1
                    continue
                del self.buffer[:size]
                frames.append(frame)
        return frames


@dataclass(frozen=True)
class Vector:
    raw: tuple[int, ...]
    values: tuple[float, ...]
    timestamp_us: int
    sequence: int
    valid: bool
    saturated: bool


@dataclass(frozen=True)
class Sample:
    state: int
    accel: Vector
    gyro: Vector
    temperature_c: float
    temperature_us: int
    temperature_valid: bool
    capture_us: int
    received_us: int
    boot_id: int
    token: int
    init_attempts: int
    read_errors: int
    missed_accel: int
    missed_gyro: int
    last_error: int


def decode(frame: bytes) -> Sample:
    if (len(frame) != 128 or frame[:4] != bytes([0x5A, 0x10, 3, 128]) or
            crc8(frame[:-1]) != frame[-1]):
        raise ValueError('invalid timed IMU frame')
    if frame[6:8] != bytes([0x1E, 0x0F]) or frame[108:111] != bytes([1, 0, 0]) or frame[126]:
        raise ValueError('unsupported BMI088 IDs/profile/reserved fields')
    flags = frame[5]
    if flags & ~31 or frame[4] not in (0, 1, 2):
        raise ValueError('invalid state/flags')
    accel_values = struct.unpack_from('<fff', frame, 72)
    gyro_values = struct.unpack_from('<fff', frame, 84)
    if not all(math.isfinite(v) for v in (*accel_values, *gyro_values)):
        raise ValueError('nonfinite sensor vector')
    return Sample(
        state=frame[4],
        accel=Vector(struct.unpack_from('<hhh', frame, 60), accel_values,
                     struct.unpack_from('<Q', frame, 36)[0],
                     struct.unpack_from('<I', frame, 28)[0], bool(flags & 1), bool(flags & 8)),
        gyro=Vector(struct.unpack_from('<hhh', frame, 66), gyro_values,
                    struct.unpack_from('<Q', frame, 44)[0],
                    struct.unpack_from('<I', frame, 32)[0], bool(flags & 2), bool(flags & 16)),
        temperature_c=struct.unpack_from('<i', frame, 96)[0] / 1000,
        temperature_us=struct.unpack_from('<Q', frame, 52)[0],
        temperature_valid=bool(flags & 4), capture_us=struct.unpack_from('<Q', frame, 100)[0],
        received_us=struct.unpack_from('<Q', frame, 118)[0],
        boot_id=struct.unpack_from('<I', frame, 114)[0],
        token=int.from_bytes(frame[111:114], 'little'),
        init_attempts=struct.unpack_from('<I', frame, 12)[0],
        read_errors=struct.unpack_from('<I', frame, 16)[0],
        missed_accel=struct.unpack_from('<I', frame, 20)[0],
        missed_gyro=struct.unpack_from('<I', frame, 24)[0],
        last_error=struct.unpack_from('<i', frame, 8)[0])


@dataclass(frozen=True)
class Timing:
    offset_ns: int  # MCU monotonic minus host monotonic
    ros_offset_ns: int
    uncertainty_ns: int

    def ros_stamp(self, device_us: int, delay_ns: int = 0) -> int:
        return device_us * 1000 - self.offset_ns + self.ros_offset_ns - delay_ns


def estimate_time(sample: Sample, send_ns: int, receive_ns: int, ros_ns: int,
                  max_rtt_ns: int) -> Timing:
    rtt = receive_ns - send_ns
    processing = (sample.capture_us - sample.received_us) * 1000
    if not 0 < rtt <= max_rtt_ns or not 0 <= processing <= rtt or ros_ns <= 0:
        raise ValueError('invalid/excessive USB timing')
    offset = ((sample.received_us * 1000 - send_ns) +
              (sample.capture_us * 1000 - receive_ns)) // 2
    return Timing(offset, ros_ns - receive_ns, (rtt - processing) // 2)


def sample_stamp(vector: Vector, sample: Sample, timing: Timing,
                 ros_ns: int, max_age_ns: int, delay_ns: int = 0) -> int:
    if (sample.state != 1 or not vector.valid or vector.saturated or vector.sequence == 0 or
            not 0 < vector.timestamp_us <= sample.capture_us or
            sample.capture_us - vector.timestamp_us > 50000):
        raise ValueError('invalid/stale/saturated sensor sample')
    stamp = timing.ros_stamp(vector.timestamp_us, delay_ns)
    if stamp <= 0 or not 0 <= ros_ns - stamp <= max_age_ns:
        raise ValueError('sensor sample is stale in ROS time')
    return stamp


def _vector(value, name):
    if (not isinstance(value, list) or len(value) != 3 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) for v in value)):
        raise ValueError(f'invalid calibration {name}')
    return tuple(float(v) for v in value)


@dataclass(frozen=True)
class Correction:
    offset: tuple[float, ...]
    scale: tuple[float, ...]
    variance: tuple[float, ...]
    temperature_c: float

    def apply(self, values):
        return tuple((v - b) * s for v, b, s in zip(values, self.offset, self.scale))


@dataclass(frozen=True)
class Calibration:
    profile_id: str
    gyro: Correction | None
    accel: Correction | None

    @classmethod
    def parse(cls, profile: dict, device_id: str):
        if (not device_id or profile.get('schema_version') != 1 or
                profile.get('kind') != 'salus_bmi088_calibration' or
                profile.get('device_id') != device_id or profile.get('frame') != 'bosch_sensor' or
                profile.get('chip_ids') != [30, 15] or profile.get('acquisition_profile') != 1 or
                not isinstance(profile.get('profile_id'), str) or not profile['profile_id']):
            raise ValueError('calibration schema/identity/configuration mismatch')
        corrections = {}
        for key in ('gyro', 'accel'):
            d = profile.get(key)
            if d is None:
                corrections[key] = None
                continue
            if not isinstance(d, dict):
                raise ValueError('invalid calibration section')
            expected = 'stationary_mean' if key == 'gyro' else 'six_position_diagonal'
            temperature = d.get('temperature_c')
            if (d.get('method') != expected or isinstance(temperature, bool) or
                    not isinstance(temperature, (int, float)) or not math.isfinite(temperature) or
                    not -40 <= temperature <= 85):
                raise ValueError('invalid calibration method/temperature')
            offset = _vector(d.get('bias_rad_s' if key == 'gyro' else 'offset_m_s2'), 'offset')
            scale = (1., 1., 1.) if key == 'gyro' else _vector(d.get('scale'), 'scale')
            variance_key = 'variance_rad2_s2' if key == 'gyro' else 'variance_m2_s4'
            variance = _vector(d.get(variance_key), 'variance')
            if (any(v < 0 for v in variance) or any(not 0.8 <= v <= 1.2 for v in scale) or
                    (key == 'gyro' and math.sqrt(sum(v*v for v in offset)) > 0.05) or
                    (key == 'accel' and any(abs(v) > 1.5 for v in offset))):
                raise ValueError('calibration coefficient outside accepted range')
            corrections[key] = Correction(offset, scale, variance, float(temperature))
        if not any(corrections.values()):
            raise ValueError('empty calibration')
        return cls(profile['profile_id'], **corrections)

    @classmethod
    def load(cls, path: str, device_id: str):
        def unique(pairs):
            d = {}
            for k, v in pairs:
                if k in d:
                    raise ValueError('duplicate calibration JSON key')
                d[k] = v
            return d

        def invalid_constant(value):
            raise ValueError('nonfinite calibration JSON: ' + value)

        profile = json.loads(Path(path).read_text(), object_pairs_hook=unique,
                             parse_constant=invalid_constant)
        if not isinstance(profile, dict):
            raise ValueError('calibration must be a JSON object')
        return cls.parse(profile, device_id)


class SequenceTracker:
    def __init__(self):
        self.session = None
        self.sequences = {}
        self.stamps = {}

    def accept(self, sample: Sample, key: str, stamp: int) -> bool:
        session = (sample.boot_id, sample.init_attempts)
        if session != self.session:
            self.session = session
            self.sequences.clear()
        vector = getattr(sample, key)
        previous = self.sequences.get(key)
        if previous is not None and (vector.sequence <= previous[0] or
                                     vector.timestamp_us <= previous[1]):
            return False
        if stamp <= self.stamps.get(key, 0):
            return False
        self.sequences[key] = (vector.sequence, vector.timestamp_us)
        self.stamps[key] = stamp
        return True
