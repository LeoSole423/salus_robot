"""Replaceable hardware transports; simulation never imports these adapters."""
from .pylontech_protocol import decode_reply, read_request


class PylontechUS2000Backend:
    """Bounded read-only polling; reopen on the next poll after a failure."""

    def __init__(self, port, baud=115200, address=2, timeout_s=0.5, serial_factory=None):
        if not port or port == 'auto':
            raise ValueError('An explicit battery serial port is required')
        if baud not in (9600, 115200) or not 0 < timeout_s <= 2:
            raise ValueError('Invalid baud rate or timeout')
        read_request(address)
        if serial_factory is None:
            import serial
            serial_factory = serial.Serial
        self._factory = serial_factory
        self._options = dict(port=port, baudrate=baud, timeout=timeout_s,
                             write_timeout=timeout_s, exclusive=True,
                             bytesize=8, parity='N', stopbits=1)
        self._address = address
        self._serial = None

    def read(self):
        try:
            if self._serial is None:
                self._serial = self._factory(**self._options)
            self._serial.reset_input_buffer()
            self._serial.write(read_request(self._address))
            reply = self._serial.read_until(b'\r', size=2048)
            return decode_reply(reply, self._address)
        except Exception:
            self.close()
            raise

    def close(self):
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None


def create_battery_backend(name, **options):
    """Add a backend here; every backend returns the same BatterySample."""
    if name != 'pylontech_us2000':
        raise ValueError(f'Unknown battery backend: {name}')
    return PylontechUS2000Backend(**options)
