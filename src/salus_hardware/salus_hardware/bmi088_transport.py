"""Exclusive read-only USB observation owner; never emits a drive command."""

from __future__ import annotations

import secrets
import time

from .bmi088_domain import FrameParser, decode, request


class ObservationClient:
    def __init__(self, port: str, timeout_s: float = 0.05, serial_factory=None):
        if not port or port == 'auto':
            raise ValueError('an explicit S3 serial path is required')
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self.serial = serial_factory(port=port, baudrate=115200, timeout=0.002,
                                     write_timeout=timeout_s, exclusive=True)
        self.serial.reset_input_buffer()
        self.parser = FrameParser()
        self.token = secrets.randbelow(0xFFFFFF)
        self.timeout_s = timeout_s

    def exchange(self):
        self.token = (self.token + 1) & 0xFFFFFF
        payload = request(self.token)
        started = time.monotonic_ns()
        if self.serial.write(payload) != len(payload):
            raise OSError('partial diagnostic write')
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            data = self.serial.read(min(max(self.serial.in_waiting, 1), 256))
            for frame in self.parser.feed(data):
                if frame[:4] != bytes([0x5A, 0x10, 3, 128]):
                    continue
                sample = decode(frame)
                if sample.token == self.token:
                    return sample, started, time.monotonic_ns()
        raise TimeoutError('timed BMI088 response unavailable')

    def close(self):
        self.serial.close()
