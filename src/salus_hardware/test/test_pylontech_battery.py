"""Characterize real stationary BMS frames and reject corrupt replies."""
import json
from pathlib import Path

import pytest

from salus_hardware.battery_backends import create_battery_backend
from salus_hardware.pylontech_protocol import decode_reply, read_request

SAMPLES = json.loads((Path(__file__).parent / 'fixtures' /
                      'pylontech_us2000_readings.json').read_text())


@pytest.mark.parametrize('row', SAMPLES)
def test_recorded_bench_frames(row):
    sample = decode_reply(row['raw_frame'].encode())
    assert sample.voltage == row['voltage_V']
    assert sample.current == -0.4
    assert sample.percentage == 0.93
    assert sample.capacity == 50
    assert sample.charge == 46.5
    assert sample.cycles == 181
    assert len(sample.cell_voltage) == 15
    assert sample.temperature == 28
    assert sample.grouped_cell_temperatures == (24, 24, 24, 24)


def reframed(body):
    return b'~' + body + f'{(-sum(body)) & 65535:04X}'.encode() + b'\r'


def test_request_only_reads_values():
    request = read_request()
    assert request[:9] == b'~20024642'
    assert int(request[-5:-1], 16) == ((-sum(request[1:-5])) & 65535)


@pytest.mark.parametrize('change', ['checksum', 'length', 'address', 'error', 'truncated'])
def test_reject_bad_frames(change):
    frame = SAMPLES[0]['raw_frame'].encode()
    body = frame[1:-5]
    if change == 'checksum':
        frame = frame[:-5] + b'0000\r'
    elif change == 'length':
        frame = reframed(body[:8] + b'C06C' + body[12:])
    elif change == 'address':
        frame = reframed(body[:2] + b'01' + body[4:])
    elif change == 'error':
        frame = reframed(body[:6] + b'01' + body[8:])
    else:
        frame = reframed(body[:-2])
    with pytest.raises(ValueError):
        decode_reply(frame)


def test_no_response_is_not_zero_charge():
    with pytest.raises(ValueError):
        decode_reply(b'')


def test_unknown_backend_rejected():
    with pytest.raises(ValueError, match='Unknown'):
        create_battery_backend('automatic')


def test_transport_reopens_after_timeout_and_never_writes_settings():
    class Port:
        def __init__(self, reply):
            self.reply, self.closed, self.sent = reply, False, []

        def reset_input_buffer(self):
            pass

        def write(self, frame):
            self.sent.append(frame)

        def read_until(self, *args, **kwargs):
            return self.reply

        def close(self):
            self.closed = True

    first = Port(b'')
    second = Port(SAMPLES[0]['raw_frame'].encode())
    ports = iter((first, second))
    backend = create_battery_backend('pylontech_us2000', port='/dev/test',
                                     serial_factory=lambda **kw: next(ports))
    with pytest.raises(ValueError):
        backend.read()
    assert first.closed
    assert backend.read().percentage == 0.93
    assert first.sent == second.sent == [read_request()]
    backend.close()
    assert second.closed
