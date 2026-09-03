"""Fixed real datum selection compatible with the legacy resolver."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Tuple

import yaml


DEFAULT_DATUM_LAT = -31.4858037
DEFAULT_DATUM_LON = -64.2410570
DEFAULT_DATUM_YAW_DEG = 0.0


def resolve_config_file_path(package_share_dir: str, filename: str) -> str:
    """Resolve a packaged config, preserving the legacy source-tree override."""
    package_share_path = Path(package_share_dir)
    default_path = package_share_path / "config" / filename
    try:
        workspace_root = package_share_path.parents[3]
        source_path = workspace_root / "src" / "navegacion_gps" / "config" / filename
        if source_path.exists():
            return str(source_path)
    except IndexError:
        pass
    return str(default_path)


def _valid_datum(lat: float, lon: float, yaw_deg: float) -> bool:
    return (
        math.isfinite(lat)
        and -90.0 <= lat <= 90.0
        and math.isfinite(lon)
        and -180.0 <= lon <= 180.0
        and math.isfinite(yaw_deg)
    )


def resolve_selected_datum(package_share_dir: str) -> Tuple[float, float, float, str]:
    """Return the selected datum or the exact legacy operational fallback."""
    datums_file = resolve_config_file_path(package_share_dir, "datums.yaml")
    fallback = (
        DEFAULT_DATUM_LAT,
        DEFAULT_DATUM_LON,
        DEFAULT_DATUM_YAW_DEG,
        datums_file,
    )
    try:
        with open(datums_file, "r", encoding="utf-8") as handle:
            document = yaml.safe_load(handle) or {}
    except Exception:
        return fallback
    if not isinstance(document, dict):
        return fallback

    selected_id = str(document.get("selected_id") or "").strip()
    datums = document.get("datums") or []
    if not selected_id or not isinstance(datums, list):
        return fallback

    for item in datums:
        if not isinstance(item, dict) or str(item.get("id") or "") != selected_id:
            continue
        try:
            lat = float(item.get("lat"))
            lon = float(item.get("lon"))
            yaw_deg = float(item.get("yaw_deg", 0.0))
        except (TypeError, ValueError):
            return fallback
        if _valid_datum(lat, lon, yaw_deg):
            return (lat, lon, yaw_deg, datums_file)
        return fallback

    return fallback


def validate_datum_override(lat: float, lon: float, yaw_deg: float) -> tuple[float, float, float]:
    """Validate explicit launch overrides using the legacy bounds."""
    values = (float(lat), float(lon), float(yaw_deg))
    if not _valid_datum(*values):
        raise ValueError(
            "datum_lat and datum_lon must be finite and in geographic range; "
            "datum_yaw_deg must be finite"
        )
    return values
