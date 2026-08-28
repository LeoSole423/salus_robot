from types import SimpleNamespace
from threading import Lock

import math

from salus_interfaces.msg import GnssRtkStatus
from salus_web.ros_gateway import CockpitRosGateway
from salus_web.ros_gateway import (
    accepts_legacy_rtk_status,
    gnss_rtk_status_payload,
)
from std_msgs.msg import String


def _status(**overrides):
    values = {
        "fix_quality": GnssRtkStatus.NO_FIX,
        "acquisition_state": GnssRtkStatus.ACQUISITION_RECEIVING,
        "delivery_backend": GnssRtkStatus.BACKEND_PIXHAWK_MAVROS,
        "delivery_state": GnssRtkStatus.DELIVERY_DELIVERING,
        "receiver_fix_type": -1,
        "satellites_visible": 255,
        "corrections_fresh": True,
        "correction_age_s": 0.25,
        "received_count": 14,
        "crc_error_count": 2,
        "source_id": "caster_primary",
        "status_detail": "corrections received; receiver has no fix",
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_gnss_rtk_projection_keeps_corrections_and_fix_quality_separate() -> None:
    payload = gnss_rtk_status_payload(_status())

    assert payload == {
        "available": True,
        "source": "gnss_rtk_status",
        "fix_quality": "no_fix",
        "acquisition_state": "receiving",
        "delivery_backend": "pixhawk_mavros",
        "delivery_state": "delivering",
        "receiver_fix_type": -1,
        "satellites_visible": None,
        "corrections_fresh": True,
        "correction_age_s": 0.25,
        "received_count": 14,
        "crc_error_count": 2,
        "source_id": "caster_primary",
        "status_detail": "corrections received; receiver has no fix",
    }


def test_gnss_rtk_projection_maps_receiver_rtk_enums_and_json_safe_unknowns() -> None:
    payload = gnss_rtk_status_payload(_status(
        fix_quality=GnssRtkStatus.RTK_FIXED,
        acquisition_state=255,
        delivery_backend=GnssRtkStatus.BACKEND_DIRECT_USB,
        delivery_state=255,
        receiver_fix_type=6,
        satellites_visible=18,
        correction_age_s=math.nan,
    ))

    assert payload["fix_quality"] == "rtk_fixed"
    assert payload["acquisition_state"] == "unknown"
    assert payload["delivery_backend"] == "direct_usb"
    assert payload["delivery_state"] == "unknown"
    assert payload["receiver_fix_type"] == 6
    assert payload["satellites_visible"] == 18
    assert payload["correction_age_s"] is None


def test_gnss_rtk_projection_rejects_negative_correction_age() -> None:
    assert gnss_rtk_status_payload(_status(correction_age_s=-0.1))["correction_age_s"] is None


def test_legacy_rtk_text_never_replaces_a_received_typed_status() -> None:
    assert accepts_legacy_rtk_status(False) is True
    assert accepts_legacy_rtk_status(True) is False


def test_gateway_keeps_typed_status_after_a_late_legacy_text_message() -> None:
    gateway = object.__new__(CockpitRosGateway)
    gateway._lock = Lock()
    gateway._cache = {}
    gateway._typed_rtk_status_received = False
    gateway._telemetry_profile = "full"
    emitted = []
    gateway._broadcast = emitted.append

    CockpitRosGateway._on_gnss_rtk_status(gateway, _status())
    CockpitRosGateway._on_rtk(gateway, String(data="legacy inferred rtk"))

    assert gateway._cache["gps_status"]["fix_quality"] == "no_fix"
    assert len(emitted) == 1
