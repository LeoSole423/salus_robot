from pathlib import Path

import pytest

from salus_hardware.camera_stream_profile import (
    STREAMS_PATH,
    StreamingHttpError,
    StreamingProfileError,
    apply_profile_xml,
    compact_http_body,
    config_from_environment,
    load_profile,
    parse_capabilities,
    parse_stream_ids,
    parse_stream_profile,
    validate_capabilities,
)
from salus_hardware.camera_stream_profile_tool import _apply, _inspect


FIXTURES = Path(__file__).parent / "fixtures/camera_stream_profiles"
STREAMS_XML = (FIXTURES / "streams.xml").read_bytes()
STREAM_XML = (FIXTURES / "stream_101.xml").read_bytes()
CAPABILITIES_XML = (FIXTURES / "stream_101_capabilities.xml").read_bytes()


class FakeStreamingClient:
    def __init__(self, *, stream_xml=STREAM_XML, capabilities=CAPABILITIES_XML):
        self.stream_xml = stream_xml
        self.capabilities = capabilities
        self.puts = []

    def get(self, path):
        if path == STREAMS_PATH:
            return STREAMS_XML
        if path.endswith("/capabilities"):
            if self.capabilities is None:
                raise StreamingHttpError(404, "Not Found", "<redacted>")
            return self.capabilities
        return self.stream_xml

    def put(self, path, body):
        self.puts.append((path, body))
        self.stream_xml = body
        return b"<ResponseStatus><statusCode>1</statusCode></ResponseStatus>"


def _profile_file(tmp_path: Path, *, width=352) -> Path:
    path = tmp_path / "profile.yaml"
    path.write_text(
        "video:\n"
        "  codec: H.264\n"
        f"  width: {width}\n"
        "  height: 288\n"
        "  fps: 15\n"
        "  rate_control: VBR\n"
        "  bitrate_kbps: 1024\n"
        "  keyframe_interval: 30\n"
        "  h264_profile: main\n"
        "audio_enabled: false\n",
        encoding="utf-8",
    )
    return path


def test_discovery_and_namespaced_optional_stream_fields() -> None:
    assert parse_stream_ids(STREAMS_XML) == ("101", "102")
    profile = parse_stream_profile(STREAM_XML, "101")
    assert profile.as_dict() == {
        "stream_id": "101", "codec": "H.264", "width": 704, "height": 576,
        "fps": 25, "rate_control": "VBR", "bitrate_kbps": 2048,
        "keyframe_interval": 50, "h264_profile": "main", "audio_enabled": False,
    }
    unavailable = parse_stream_profile(STREAMS_XML, "102").as_dict()
    assert unavailable["stream_id"] == "102"
    assert unavailable["codec"] == "unavailable"
    assert unavailable["fps"] == "unavailable"


def test_capabilities_parse_values_ranges_and_units() -> None:
    capabilities = parse_capabilities(CAPABILITIES_XML)
    assert capabilities is not None
    assert capabilities.allowed["width"] == frozenset({"704", "352"})
    assert capabilities.allowed["fps"] == frozenset({"25", "15"})
    assert capabilities.ranges["bitrate_kbps"] == (128.0, 4096.0)
    validate_capabilities({"width": 352, "fps": 15}, capabilities)


def test_apply_preserves_unknown_xml_and_changes_only_managed_fields(tmp_path: Path) -> None:
    desired = load_profile(_profile_file(tmp_path))
    changed, changes = apply_profile_xml(STREAM_XML, desired, "101")
    assert set(changes) == {
        "width",
        "height",
        "fps",
        "bitrate_kbps",
        "keyframe_interval",
    }
    assert b"futureVendorField" in changed
    assert b"preserve-me" in changed
    assert parse_stream_profile(changed, "101").width == 352
    assert parse_stream_profile(changed, "101").fps == 15


def test_dry_run_does_not_put_and_reports_semantic_changes(tmp_path: Path) -> None:
    client = FakeStreamingClient()
    result = _apply(client, "101", str(_profile_file(tmp_path)), dry_run=True)
    assert result["dry_run"] is True
    assert result["put"] is False
    assert result["changes"]["width"] == {"old": 704, "new": 352}
    assert client.puts == []


def test_identical_profile_is_idempotent_without_put(tmp_path: Path) -> None:
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        "video:\n"
        "  codec: H.264\n"
        "  width: 704\n"
        "  height: 576\n"
        "  fps: 25\n"
        "  rate_control: VBR\n"
        "  bitrate_kbps: 2048\n"
        "  keyframe_interval: 50\n"
        "  h264_profile: main\n"
        "audio_enabled: false\n",
        encoding="utf-8",
    )
    client = FakeStreamingClient()
    result = _apply(client, "101", str(profile), dry_run=False)
    assert result["put"] is False
    assert "verified" not in result
    assert client.puts == []


def test_apply_puts_full_xml_and_verifies_with_get(tmp_path: Path) -> None:
    client = FakeStreamingClient()
    result = _apply(client, "101", str(_profile_file(tmp_path)), dry_run=False)
    assert result["put"] is True
    assert result["verified"]["width"] == 352
    assert len(client.puts) == 1
    assert client.puts[0][0] == f"{STREAMS_PATH}/101"


def test_capability_contradiction_fails_closed() -> None:
    capabilities = parse_capabilities(CAPABILITIES_XML)
    with pytest.raises(StreamingProfileError, match="outside camera capabilities"):
        validate_capabilities({"width": 999}, capabilities)


def test_inspect_reports_capabilities_without_credentials() -> None:
    result = _inspect(FakeStreamingClient(), "101")
    serialized = str(result)
    assert result["stream_ids"] == ["101", "102"]
    assert result["profile"]["stream_id"] == "101"
    assert "password" not in serialized.lower()
    assert "Authorization" not in serialized


def test_http_error_and_environment_credentials_are_safe(tmp_path: Path) -> None:
    secret_file = tmp_path / "camera.pass"
    secret_file.write_text("secret-value\n", encoding="utf-8")
    config = config_from_environment({
        "CAMERA_HOST": "camera.local", "CAMERA_PORT": "80", "CAMERA_USER": "operator",
        "CAMERA_PASS_FILE": str(secret_file),
    })
    assert config.password == "secret-value"
    body = compact_http_body(
        b"operator secret-value ResponseStatus denied", "operator", config.password
    )
    assert "operator" not in body
    assert "secret-value" not in body
    error = StreamingHttpError(403, "Forbidden", body)
    assert "403" in str(error)


def test_tool_is_manual_and_does_not_change_ros_runtime() -> None:
    source = Path(__file__).parents[0].parent / "salus_hardware/camera_stream_profile_tool.py"
    text = source.read_text(encoding="utf-8")
    assert "rclpy" not in text
    assert "camera_stream_profile_tool" in (
        Path(__file__).parents[0].parent / "setup.py"
    ).read_text(encoding="utf-8")
