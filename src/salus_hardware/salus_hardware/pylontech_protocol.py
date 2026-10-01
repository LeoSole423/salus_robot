"""Pure Pylontech low-voltage RS485 ASCII framing and US2000 data parser."""
import struct

from .battery_domain import BatterySample


def read_request(address: int = 2) -> bytes:
    """Read all modules; only CID2=42 is emitted by this adapter."""
    if not 0 <= address <= 255:
        raise ValueError('Address must be in [0, 255]')
    body = f'20{address:02X}4642E002FF'.encode('ascii')
    return b'~' + body + f'{(-sum(body)) & 65535:04X}'.encode('ascii') + b'\r'


def decode_reply(frame: bytes, address: int = 2) -> BatterySample:
    """Reject malformed, foreign, failed or unsupported multi-module replies."""
    try:
        if len(frame) < 18 or frame[:1] != b'~' or frame[-1:] != b'\r':
            raise ValueError('Invalid delimiters')
        body, checksum = frame[1:-5], int(frame[-5:-1], 16)
        if checksum != ((-sum(body)) & 65535):
            raise ValueError('Checksum mismatch')
        header = bytes.fromhex(body[:8].decode('ascii'))
        if header != bytes((0x20, address, 0x46, 0)):
            raise ValueError('Unexpected version, address, CID1 or return code')
        length = int(body[8:12], 16)
        n = length & 4095
        if n != len(body[12:]) or n % 2:
            raise ValueError('Length mismatch')
        if (sum((n >> k) & 15 for k in (0, 4, 8)) + (length >> 12)) & 15:
            raise ValueError('Length checksum mismatch')
        data = bytes.fromhex(body[12:].decode('ascii'))
        if data[1] != 1:
            raise ValueError('Only single-module US2000 is supported')
        p = 2
        count = data[p]
        p += 1
        if count != 15:
            raise ValueError('US2000 requires 15 cells')
        cells = struct.unpack_from('>15H', data, p)
        p += 30
        nt = data[p]
        p += 1
        if not 1 <= nt <= 16:
            raise ValueError('Invalid temperature count')
        temps = struct.unpack_from(f'>{nt}H', data, p)
        p += 2 * nt
        current, voltage, charge = struct.unpack_from('>hHH', data, p)
        p += 6
        items = data[p]
        p += 1
        capacity, cycles = struct.unpack_from('>HH', data, p)
        p += 4
        if items not in (2, 4):
            raise ValueError('Unsupported capacity fields')
        if items == 4:
            charge = int.from_bytes(data[p:p + 3], 'big')
            capacity = int.from_bytes(data[p + 3:p + 6], 'big')
            p += 6
        if p != len(data):
            raise ValueError('Truncated or extra payload')
        if not capacity or not 0 <= charge <= capacity:
            raise ValueError('Invalid capacity')
        if not 20000 <= voltage <= 65000 or any(not 1000 <= c <= 5000 for c in cells):
            raise ValueError('Invalid voltage')
        temperatures = tuple((t - 2731) / 10 for t in temps)
        if any(not -50 <= t <= 120 for t in temperatures):
            raise ValueError('Invalid temperature')
        return BatterySample(voltage / 1000, current / 10, charge / 1000,
                             capacity / 1000, tuple(c / 1000 for c in cells),
                             temperatures[1:], temperatures[0], cycles)
    except (IndexError, struct.error, UnicodeError) as exc:
        raise ValueError('Malformed Pylontech reply') from exc
