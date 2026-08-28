"""Pure selection rules for one explicitly configured physical GNSS source."""

from __future__ import annotations

from dataclasses import dataclass
import math

from sensor_msgs.msg import NavSatFix, NavSatStatus


PRIMARY_SOURCE_ID = "gnss_primary"
SECONDARY_SOURCE_ID = "gnss_secondary"
SUPPORTED_SOURCE_IDS = (PRIMARY_SOURCE_ID, SECONDARY_SOURCE_ID)


@dataclass(frozen=True)
class GnssSelectionDecision:
    accepted: bool
    reason: str


class GnssSelectionPolicy:
    """Select one GNSS identity without fallback and require monotonic samples."""

    def __init__(self, selected_source: object, expected_frame: object) -> None:
        selected = str(selected_source).strip().lower()
        if selected not in SUPPORTED_SOURCE_IDS:
            raise ValueError("selected_source must be gnss_primary or gnss_secondary")
        frame = str(expected_frame).strip()
        if not frame:
            raise ValueError("expected_frame must not be empty")
        self.selected_source = selected
        self.expected_frame = frame
        self._last_stamp_ns: int | None = None

    def evaluate(
        self, source_id: object, message: NavSatFix
    ) -> GnssSelectionDecision:
        if str(source_id).strip().lower() != self.selected_source:
            return GnssSelectionDecision(False, "source_not_selected")
        if message.header.frame_id != self.expected_frame:
            return GnssSelectionDecision(False, "unexpected_frame")
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns <= 0 or not 0 <= int(message.header.stamp.nanosec) < 1_000_000_000:
            return GnssSelectionDecision(False, "invalid_timestamp")
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            return GnssSelectionDecision(False, "non_monotonic_timestamp")
        if not all(math.isfinite(float(value)) for value in message.position_covariance):
            return GnssSelectionDecision(False, "non_finite_covariance")
        if int(message.status.status) != NavSatStatus.STATUS_NO_FIX:
            position = (message.latitude, message.longitude, message.altitude)
            if not all(math.isfinite(float(value)) for value in position):
                return GnssSelectionDecision(False, "non_finite_position")
            if not -90.0 <= float(message.latitude) <= 90.0:
                return GnssSelectionDecision(False, "latitude_out_of_range")
            if not -180.0 <= float(message.longitude) <= 180.0:
                return GnssSelectionDecision(False, "longitude_out_of_range")
        self._last_stamp_ns = stamp_ns
        return GnssSelectionDecision(True, "accepted")
