"""Read-only, sanitized NTRIP source configuration."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import yaml


_WHITESPACE_OR_CONTROL = re.compile(r"[\x00-\x20\x7f]")


@dataclass(frozen=True)
class NtripSource:
    """One NTRIP endpoint; credentials are intentionally absent from repr."""

    id: str
    label: str
    host: str
    port: int
    mountpoint: str
    username: str = field(repr=False)
    password: str = field(repr=False)


def _text(value: object, *, default: str = "") -> str:
    if value is None:
        return default
    if not isinstance(value, str):
        raise ValueError("invalid_rtk_source")
    return value.strip()


def validate_source(source: NtripSource) -> None:
    if (
        not source.id
        or not source.label
        or not source.host
        or not source.mountpoint
        or isinstance(source.port, bool)
        or not isinstance(source.port, int)
        or not 1 <= source.port <= 65535
    ):
        raise ValueError("invalid_rtk_source")
    for value in (source.id, source.label, source.username, source.password):
        if not isinstance(value, str) or any(
            ord(char) < 32 or ord(char) == 127 for char in value
        ):
            raise ValueError("invalid_rtk_source")
    if not isinstance(source.host, str) or _WHITESPACE_OR_CONTROL.search(source.host):
        raise ValueError("invalid_rtk_endpoint")
    if any(char in source.host for char in "/\\@?#"):
        raise ValueError("invalid_rtk_endpoint")
    if not isinstance(source.mountpoint, str) or _WHITESPACE_OR_CONTROL.search(
        source.mountpoint
    ):
        raise ValueError("invalid_rtk_endpoint")
    if any(char in source.mountpoint for char in "\\?#"):
        raise ValueError("invalid_rtk_endpoint")
    if "://" in source.host or "://" in source.mountpoint:
        raise ValueError("invalid_rtk_endpoint")


def _parse_source(raw: object) -> NtripSource:
    if not isinstance(raw, dict):
        raise ValueError("invalid_rtk_source")
    source_id = _text(raw.get("id"))
    password = raw.get("password", "")
    if not isinstance(password, str):
        raise ValueError("invalid_rtk_source")
    source = NtripSource(
        id=source_id,
        label=_text(raw.get("label"), default=source_id),
        host=_text(raw.get("host")),
        port=raw.get("port", 2101),
        mountpoint=_text(raw.get("mountpoint")),
        username=_text(raw.get("username")),
        password=password,
    )
    validate_source(source)
    return source


def load_sources(path: Path) -> tuple[list[NtripSource], str]:
    """Load a YAML file without ever writing it or exposing its path/secrets."""

    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if not isinstance(document, dict):
            raise ValueError("invalid_rtk_sources_config")
        raw_sources = document.get("sources", [])
        if not isinstance(raw_sources, list):
            raise ValueError("invalid_rtk_sources_config")
        sources = [_parse_source(raw) for raw in raw_sources]
        if not sources or len({source.id for source in sources}) != len(sources):
            raise ValueError("invalid_rtk_sources_config")
        active = _text(document.get("active_source_id"))
        if active and active not in {source.id for source in sources}:
            raise ValueError("invalid_rtk_sources_config")
        return sources, active
    except (OSError, TypeError, ValueError, KeyError, AttributeError, yaml.YAMLError):
        raise ValueError("invalid_or_unreadable_rtk_sources_config") from None


def validate_positive_finite(value: object, name: str) -> float:
    """Validate a ROS duration parameter without leaking its raw value."""

    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{name} must be positive and finite") from None
    if not math.isfinite(result) or result <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return result
