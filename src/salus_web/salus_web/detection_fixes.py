"""Bounded, atomic handoff of GPS fixes to the independent detector."""
from collections import deque
import json
import math
from pathlib import Path
import tempfile


class DetectionFixHistory:
    def __init__(self, path):
        self.path = Path(path)
        self.fixes = deque(maxlen=256)
        self.changed = False

    def observe(self, timestamp_ms, status, lat, lon):
        if (status < 0 or not math.isfinite(lat) or not math.isfinite(lon)
                or abs(lat) > 90 or abs(lon) > 180):
            return
        while self.fixes and self.fixes[0]["timestamp_ms"] < timestamp_ms - 8000:
            self.fixes.popleft()
        self.fixes.append({"timestamp_ms": timestamp_ms, "lat": lat, "lon": lon})
        self.changed = True

    def flush(self):
        if not self.changed:
            return
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode="w", dir=self.path.parent, delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(list(self.fixes), stream, allow_nan=False)
            temporary.replace(self.path)
            self.changed = False
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
