"""Hikvision streaming profile inspection and safe XML mutation."""

from __future__ import annotations

from dataclasses import dataclass, field as dataclass_field
import math
import os
from pathlib import Path
from typing import Any, Callable, Mapping
from urllib.error import HTTPError, URLError
from urllib.request import (
    HTTPDigestAuthHandler,
    HTTPPasswordMgrWithDefaultRealm,
    Request,
    build_opener,
)
from xml.etree import ElementTree

import yaml


STREAMS_PATH = "/ISAPI/Streaming/channels"
_FIELD_ALIASES = {
    "codec": ("videoCodecType", "codec"),
    "width": ("videoResolutionWidth", "width"),
    "height": ("videoResolutionHeight", "height"),
    "fps": ("maxFrameRate", "frameRate", "fps"),
    "rate_control": ("videoQualityControlType", "rateControl", "rate_control"),
    "bitrate_kbps": ("vbrUpperCap", "constantBitRate", "bitRate", "bitrate"),
    "gop_length_frames": ("GovLength",),
    "h264_profile": ("H264Profile", "h264Profile", "profile"),
    "audio_enabled": ("audioEnabled",),
}
_ALLOWED_FIELDS = frozenset(_FIELD_ALIASES)
_STRING_FIELDS = frozenset({"codec", "rate_control", "h264_profile"})
_NUMERIC_FIELDS = frozenset({"width", "height", "fps", "bitrate_kbps", "gop_length_frames"})


class StreamingProfileError(ValueError):
    """A profile is invalid or cannot be safely applied."""


class StreamingHttpError(RuntimeError):
    """HTTP failure with a compact, credential-free diagnostic."""

    def __init__(self, status: int, reason: str, body: str) -> None:
        self.status = status
        self.reason = reason
        self.body = body
        super().__init__(f"ISAPI HTTP {status} {reason}; body='{body}'")


@dataclass(frozen=True)
class ResponseStatus:
    """Application-level status returned by a successful ISAPI HTTP PUT."""

    status_code: str
    status_string: str
    sub_status_code: str
    m_err_code: str | None = None

    def is_success(self) -> bool:
        return (
            self.status_code in {"0", "1"}
            and self.status_string.casefold() == "ok"
            and self.sub_status_code.casefold() in {"ok", "success"}
        )

    def as_dict(self, redact: Callable[[str], str] | None = None) -> dict[str, str]:
        clean = redact or (lambda value: value)
        result = {
            "statusCode": clean(self.status_code),
            "statusString": clean(self.status_string),
            "subStatusCode": clean(self.sub_status_code),
        }
        if self.m_err_code is not None:
            result["MErrCode"] = clean(self.m_err_code)
        return result

    def diagnostic(self, redact: Callable[[str], str] | None = None) -> str:
        fields = self.as_dict(redact)
        return "; ".join(f"{key}={value}" for key, value in fields.items())


@dataclass(frozen=True)
class StreamingClientConfig:
    host: str
    port: int
    username: str
    password: str
    timeout_s: float = 3.0

    def __post_init__(self) -> None:
        if not self.host or not self.username or not self.password:
            raise StreamingProfileError(
                "CAMERA_HOST, CAMERA_USER and camera password are required"
            )
        if not 1 <= self.port <= 65535:
            raise StreamingProfileError("CAMERA_PORT must be between 1 and 65535")
        if not 0.1 <= self.timeout_s <= 10.0:
            raise StreamingProfileError("camera timeout must be between 0.1 and 10 seconds")


@dataclass(frozen=True)
class StreamingProfile:
    stream_id: str
    codec: str | None = None
    width: int | None = None
    height: int | None = None
    fps: float | None = None
    rate_control: str | None = None
    bitrate_kbps: int | None = None
    gop_length_frames: int | None = None
    h264_profile: str | None = None
    audio_enabled: bool | None = None

    def as_dict(self) -> dict[str, Any]:
        values = {
                "stream_id": self.stream_id,
                "codec": self.codec,
                "width": self.width,
                "height": self.height,
                "fps": self.fps,
                "rate_control": self.rate_control,
                "bitrate_kbps": self.bitrate_kbps,
                "gop_length_frames": self.gop_length_frames,
                "h264_profile": self.h264_profile,
                "audio_enabled": self.audio_enabled,
        }
        return {
            key: value if value is not None else "unavailable"
            for key, value in values.items()
        }


@dataclass(frozen=True)
class StreamingCapabilities:
    allowed: Mapping[str, frozenset[str]]
    ranges: Mapping[str, tuple[float, float]]
    bitrate_allowed: Mapping[str, frozenset[str]] = dataclass_field(default_factory=dict)
    bitrate_ranges: Mapping[str, tuple[float, float]] = dataclass_field(default_factory=dict)

    def as_dict(self) -> dict[str, Any]:
        return {
            "allowed": {key: sorted(values) for key, values in self.allowed.items()},
            "ranges": {key: list(values) for key, values in self.ranges.items()},
            "bitrate_by_rate_control": {
                mode: {
                    "allowed": sorted(self.bitrate_allowed[mode])
                    if mode in self.bitrate_allowed else [],
                    "range": list(self.bitrate_ranges[mode])
                    if mode in self.bitrate_ranges else "unavailable",
                }
                for mode in sorted(set(self.bitrate_allowed) | set(self.bitrate_ranges))
            },
        }


def config_from_environment(environ: Mapping[str, str] | None = None) -> StreamingClientConfig:
    """Load the same camera variables used by ``camera_node``."""
    values = os.environ if environ is None else environ
    password_file = values.get("CAMERA_PASS_FILE", "").strip()
    password = (
        Path(password_file).read_text(encoding="utf-8").strip()
        if password_file else values.get("CAMERA_PASS", "").strip()
    )
    try:
        port = int(values.get("CAMERA_PORT", "80") or "80")
    except ValueError as error:
        raise StreamingProfileError("CAMERA_PORT must be an integer") from error
    return StreamingClientConfig(
        values.get("CAMERA_HOST", "").strip(),
        port,
        values.get("CAMERA_USER", "").strip(),
        password,
    )


class HikvisionStreamingClient:
    """Small Digest-auth client for the camera streaming API."""

    def __init__(self, config: StreamingClientConfig) -> None:
        self._config = config
        base = f"http://{config.host}:{config.port}"
        manager = HTTPPasswordMgrWithDefaultRealm()
        manager.add_password(None, base, config.username, config.password)
        self._opener = build_opener(HTTPDigestAuthHandler(manager))

    def get(self, path: str) -> bytes:
        return self._request("GET", path)

    def put(self, path: str, body: bytes) -> bytes:
        return self._request("PUT", path, body)

    def redact_text(self, value: str) -> str:
        return compact_http_body(
            value.encode("utf-8"),
            self._config.username,
            self._config.password,
            max_len=160,
        )

    def _request(self, method: str, path: str, body: bytes | None = None) -> bytes:
        if not path.startswith("/"):
            raise StreamingProfileError("ISAPI path must be absolute")
        request = Request(
            f"http://{self._config.host}:{self._config.port}{path}",
            data=body,
            method=method,
        )
        if body is not None:
            request.add_header("Content-Type", "application/xml")
        try:
            with self._opener.open(request, timeout=self._config.timeout_s) as response:
                return response.read()
        except HTTPError as error:
            compact = compact_http_body(error.read(), self._config.username, self._config.password)
            raise StreamingHttpError(error.code, error.reason, compact) from error
        except (URLError, TimeoutError, OSError) as error:
            raise RuntimeError(f"ISAPI request failed: {error}") from error


def compact_http_body(
    body: bytes,
    username: str = "",
    password: str = "",
    max_len: int = 280,
) -> str:
    """Keep diagnostics useful without leaking credentials or XML dumps."""
    compact = " ".join(body.decode("utf-8", "replace").split())
    for secret in (username, password):
        if secret:
            compact = compact.replace(secret, "<redacted>")
    if not compact:
        return "<empty>"
    return compact if len(compact) <= max_len else compact[:max_len - 3] + "..."


def parse_response_status(xml: bytes | str | None) -> ResponseStatus:
    """Parse the application result returned by a Hikvision ISAPI PUT."""
    if xml is None:
        raise StreamingProfileError("PUT ResponseStatus is empty")
    text = xml.decode("utf-8", "replace") if isinstance(xml, bytes) else xml
    if not text.strip():
        raise StreamingProfileError("PUT ResponseStatus is empty")
    try:
        root = _parse_xml(xml)
    except StreamingProfileError as error:
        raise StreamingProfileError("PUT ResponseStatus is malformed") from error
    response = root if _local_name(root.tag) == "ResponseStatus" else next(
        iter(_elements(root, "ResponseStatus")), None
    )
    if response is None:
        raise StreamingProfileError("PUT response is not a ResponseStatus")

    def required(name: str) -> str:
        element = next(
            (item for item in response.iter() if _local_name(item.tag) == name),
            None,
        )
        value = (element.text or "").strip() if element is not None else ""
        if not value:
            raise StreamingProfileError(f"PUT ResponseStatus missing {name}")
        return value

    m_err = next(
        (item for item in response.iter() if _local_name(item.tag) == "MErrCode"),
        None,
    )
    return ResponseStatus(
        status_code=required("statusCode"),
        status_string=required("statusString"),
        sub_status_code=required("subStatusCode"),
        m_err_code=(m_err.text or "").strip() if m_err is not None else None,
    )


def parse_stream_ids(xml: bytes | str) -> tuple[str, ...]:
    root = _parse_xml(xml)
    result = []
    for stream in _elements(root, "StreamingChannel"):
        value = _child_text(stream, "id")
        if value and value not in result:
            result.append(value)
    return tuple(result)


def parse_stream_profile(xml: bytes | str, stream_id: str) -> StreamingProfile:
    root = _parse_xml(xml)
    stream = _find_stream(root, stream_id)
    values: dict[str, Any] = {"stream_id": stream_id}
    for field, aliases in _FIELD_ALIASES.items():
        if field == "bitrate_kbps":
            aliases = _bitrate_aliases(values.get("rate_control"))
        if not aliases:
            continue
        element = _field_element(stream, field, aliases)
        if element is None or not (element.text or "").strip():
            continue
        try:
            values[field] = _parse_field(field, element)
        except ValueError as error:
            raise StreamingProfileError(f"invalid {field} in stream {stream_id}") from error
    return StreamingProfile(**values)


def parse_capabilities(xml: bytes | str | None) -> StreamingCapabilities | None:
    if xml is None:
        return None
    root = _parse_xml(xml)
    allowed: dict[str, frozenset[str]] = {}
    ranges: dict[str, tuple[float, float]] = {}
    bitrate_allowed: dict[str, frozenset[str]] = {}
    bitrate_ranges: dict[str, tuple[float, float]] = {}
    for field, aliases in _FIELD_ALIASES.items():
        if field == "bitrate_kbps":
            continue
        elements = [element for alias in aliases for element in _elements(root, alias)]
        options, bounds = _parse_capability_elements(field, elements)
        if options:
            allowed[field] = frozenset(options)
        if len(bounds) >= 2:
            ranges[field] = (min(bounds), max(bounds))
    for mode, alias in (("CBR", "constantBitRate"), ("VBR", "vbrUpperCap")):
        options, bounds = _parse_capability_elements(
            "bitrate_kbps",
            _elements(root, alias),
        )
        if options:
            bitrate_allowed[mode] = frozenset(options)
        if len(bounds) >= 2:
            bitrate_ranges[mode] = (min(bounds), max(bounds))
    return StreamingCapabilities(allowed, ranges, bitrate_allowed, bitrate_ranges)


def _parse_capability_elements(
    field: str,
    elements: list[ElementTree.Element],
) -> tuple[set[str], list[float]]:
    options: set[str] = set()
    bounds: list[float] = []
    for element in elements:
        for child in element.iter():
            opt = (child.attrib.get("opt") or "").strip()
            if opt:
                options.update(_normalize_options(field, opt, child))
            if child is not element and _local_name(child.tag).lower() == "option":
                value = (child.text or "").strip()
                if value:
                    options.update(_normalize_options(field, value, child))
        for attribute in ("min", "minimum", "max", "maximum"):
            if attribute in element.attrib:
                try:
                    bounds.append(float(element.attrib[attribute]))
                except ValueError:
                    pass
    return options, bounds


def apply_profile_xml(
    xml: bytes | str,
    desired: Mapping[str, Any],
    stream_id: str,
) -> tuple[bytes, dict[str, dict[str, Any]]]:
    """Mutate only managed fields and return XML plus semantic changes."""
    normalized = normalize_profile_fields(desired)
    root = _parse_xml(xml)
    stream = _find_stream(root, stream_id)
    before = parse_stream_profile(xml, stream_id)
    effective_rate_control = normalized.get("rate_control", before.rate_control)
    changes: dict[str, dict[str, Any]] = {}
    for field, value in normalized.items():
        aliases = _FIELD_ALIASES[field]
        if field == "bitrate_kbps":
            aliases = _bitrate_aliases(effective_rate_control)
        if not aliases:
            raise StreamingProfileError(
                "bitrate_kbps requires rate_control CBR or VBR"
            )
        element = _field_element(stream, field, aliases)
        if element is None:
            raise StreamingProfileError(f"stream {stream_id} does not expose field {field}")
        old = _parse_field(field, element) if field == "bitrate_kbps" else getattr(before, field)
        encoded = _encode_field(field, value, element)
        if old != value:
            changes[field] = {"old": old, "new": value}
            element.text = encoded
    body = ElementTree.tostring(root, encoding="utf-8")
    return body, changes


def normalize_profile_fields(values: Mapping[str, Any]) -> dict[str, Any]:
    unknown = set(values) - _ALLOWED_FIELDS
    if unknown:
        raise StreamingProfileError(f"unsupported profile fields: {', '.join(sorted(unknown))}")
    result: dict[str, Any] = {}
    for field, value in values.items():
        if value is None:
            continue
        if field in _STRING_FIELDS:
            if not isinstance(value, str) or not value.strip():
                raise StreamingProfileError(f"{field} must be a non-empty string")
            result[field] = value.strip()
        elif field in _NUMERIC_FIELDS:
            number = float(value)
            if not math.isfinite(number) or number <= 0:
                raise StreamingProfileError(f"{field} must be finite and positive")
            result[field] = int(number) if field != "fps" or number.is_integer() else number
        elif field == "audio_enabled":
            if not isinstance(value, bool):
                raise StreamingProfileError("audio_enabled must be boolean")
            result[field] = value
    if not result:
        raise StreamingProfileError("profile must specify at least one managed field")
    return result


def validate_capabilities(
    desired: Mapping[str, Any],
    capabilities: StreamingCapabilities | None,
    current_rate_control: str | None = None,
) -> None:
    if capabilities is None:
        return
    normalized = normalize_profile_fields(desired)
    effective_rate_control = normalized.get("rate_control", current_rate_control)
    bitrate_mode = str(effective_rate_control or "").strip().upper()
    if "bitrate_kbps" in normalized and bitrate_mode not in {"CBR", "VBR"}:
        raise StreamingProfileError(
            "bitrate_kbps requires effective rate_control CBR or VBR"
        )
    for field, value in normalized.items():
        if field == "bitrate_kbps":
            allowed = capabilities.bitrate_allowed.get(bitrate_mode, frozenset())
            bounds = capabilities.bitrate_ranges.get(bitrate_mode)
        else:
            allowed = capabilities.allowed.get(field, frozenset())
            bounds = capabilities.ranges.get(field)
        key = _capability_value(field, value)
        if allowed and key not in allowed:
            raise StreamingProfileError(f"{field}={value} is outside camera capabilities")
        if bounds and not bounds[0] <= float(value) <= bounds[1]:
            raise StreamingProfileError(f"{field}={value} is outside camera capabilities")


def verify_profile(profile: StreamingProfile, desired: Mapping[str, Any]) -> None:
    for field, value in normalize_profile_fields(desired).items():
        if getattr(profile, field) != value:
            raise StreamingProfileError(f"GET verification failed for {field}")


def load_profile(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}
    if not isinstance(document, Mapping):
        raise StreamingProfileError("profile root must be a mapping")
    video = document.get("video", {})
    if not isinstance(video, Mapping):
        raise StreamingProfileError("profile video must be a mapping")
    values = dict(video)
    if "audio_enabled" in document:
        values["audio_enabled"] = document["audio_enabled"]
    return normalize_profile_fields(values)


def _parse_xml(xml: bytes | str) -> ElementTree.Element:
    try:
        return ElementTree.fromstring(xml)
    except ElementTree.ParseError as error:
        raise StreamingProfileError("invalid streaming XML") from error


def _elements(root: ElementTree.Element, local_name: str) -> list[ElementTree.Element]:
    return [element for element in root.iter() if _local_name(element.tag) == local_name]


def _find_stream(root: ElementTree.Element, stream_id: str) -> ElementTree.Element:
    for stream in _elements(root, "StreamingChannel"):
        if _child_text(stream, "id") == str(stream_id):
            return stream
    if _local_name(root.tag) == "StreamingChannel" and _child_text(root, "id") == str(stream_id):
        return root
    raise StreamingProfileError(f"stream id {stream_id} not found")


def _first_descendant(
    root: ElementTree.Element,
    aliases: tuple[str, ...],
) -> ElementTree.Element | None:
    for alias in aliases:
        for element in root.iter():
            if _local_name(element.tag) == alias:
                return element
    return None


def _bitrate_aliases(rate_control: Any) -> tuple[str, ...]:
    normalized = str(rate_control or "").strip().upper()
    if normalized == "CBR":
        return ("constantBitRate",)
    if normalized == "VBR":
        return ("vbrUpperCap",)
    return ()


def _field_element(
    root: ElementTree.Element,
    field: str,
    aliases: tuple[str, ...],
) -> ElementTree.Element | None:
    element = _first_descendant(root, aliases)
    if element is not None or field != "audio_enabled":
        return element
    for parent in root.iter():
        if _local_name(parent.tag).lower() in {"audio", "audiochannel"}:
            for child in parent.iter():
                if _local_name(child.tag).lower() == "enabled":
                    return child
    return None


def _child_text(root: ElementTree.Element, name: str) -> str:
    child = next((item for item in list(root) if _local_name(item.tag) == name), None)
    return (child.text or "").strip() if child is not None else ""


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _parse_field(field: str, element: ElementTree.Element) -> Any:
    text = (element.text or "").strip()
    if field in _STRING_FIELDS:
        return text
    if field == "audio_enabled":
        return text.lower() in {"true", "1", "yes", "on"}
    value = float(text)
    if field == "fps":
        unit = (element.attrib.get("unit") or "").lower()
        if unit in {"centihz", "hundredths", "0.01fps", "1/100fps"}:
            value /= 100.0
        elif not unit and value > 120.0:
            value /= 100.0
        return int(value) if value.is_integer() else value
    if field == "bitrate_kbps":
        return int(value)
    return int(value) if value.is_integer() else value


def _normalize_options(field: str, text: str, element: ElementTree.Element) -> set[str]:
    result = set()
    for token in text.replace(",", " ").split():
        try:
            value = _parse_scalar(field, token, element)
            result.add(_capability_value(field, value))
        except ValueError:
            result.add(token.strip().lower() if field in _STRING_FIELDS else token.strip())
    return result


def _parse_scalar(field: str, text: str, element: ElementTree.Element) -> Any:
    clone = ElementTree.Element("value", element.attrib)
    clone.text = text
    return _parse_field(field, clone)


def _capability_value(field: str, value: Any) -> str:
    if field in _STRING_FIELDS:
        return str(value).strip().lower()
    if field == "audio_enabled":
        return str(bool(value)).lower()
    return str(value)


def _encode_field(field: str, value: Any, element: ElementTree.Element) -> str:
    if field == "fps":
        unit = (element.attrib.get("unit") or "").lower()
        if unit in {"centihz", "hundredths", "0.01fps", "1/100fps"} or (
            not unit
            and 1.2 < float(value) <= 120.0
            and float(element.text or 0) > 120.0
        ):
            return str(int(round(float(value) * 100)))
    if field == "audio_enabled":
        return "true" if value else "false"
    if field in _NUMERIC_FIELDS:
        return str(int(value)) if float(value).is_integer() else str(value)
    return str(value)
