from pathlib import Path

import pytest

from salus_hardware.ntrip_source_config import NtripSource, load_sources, validate_source


def test_config_is_read_only_and_repr_does_not_contain_credentials(tmp_path: Path) -> None:
    config = tmp_path / "sources.yaml"
    original = (
        "active_source_id: test\n"
        "sources:\n"
        "  - id: test\n"
        "    label: Test\n"
        "    host: 127.0.0.1\n"
        "    port: 2101\n"
        "    mountpoint: RTCM3\n"
        "    username: private-user\n"
        "    password: private-password\n"
    )
    config.write_text(original, encoding="utf-8")
    sources, active = load_sources(config)
    assert active == "test"
    assert sources[0].username == "private-user"
    assert "private-user" not in repr(sources[0])
    assert "private-password" not in repr(sources[0])
    assert config.read_text(encoding="utf-8") == original


def test_invalid_config_errors_are_sanitized(tmp_path: Path) -> None:
    config = tmp_path / "secret-private-path.yaml"
    config.write_text(
        "active_source_id: test\n"
        "sources:\n"
        "  - id: test\n"
        "    label: Test\n"
        "    host: caster.example/path\n"
        "    port: 2101\n"
        "    mountpoint: RTCM3\n"
        "    username: user\n"
        "    password: secret\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError) as error:
        load_sources(config)
    assert str(error.value) == "invalid_or_unreadable_rtk_sources_config"
    assert "secret" not in str(error.value)
    assert str(config) not in str(error.value)


@pytest.mark.parametrize(
    "host,mountpoint",
    (("caster\r\nHost: evil", "RTCM3"), ("caster", "RTCM3?x=1"), ("caster", "RTCM3\r\n")),
)
def test_endpoint_injection_is_rejected(host: str, mountpoint: str) -> None:
    source = NtripSource("id", "label", host, 2101, mountpoint, "user", "password")
    with pytest.raises(ValueError, match="invalid_rtk_endpoint"):
        validate_source(source)
