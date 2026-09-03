import pytest

from salus_hardware.ntrip_protocol import (
    ChunkDecoder,
    RtcmDecoder,
    parse_response,
)
from salus_hardware.rtk_domain import crc24q


def _frame(payload: bytes = b"\x3e\x00") -> bytes:
    header = bytes((0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF))
    body = header + payload
    return body + crc24q(body).to_bytes(3, "big")


def test_known_crc_and_fragmented_rtcm_resynchronization() -> None:
    assert crc24q(b"123456789") == 0xCDE703
    valid = _frame()
    corrupt = valid[:-1] + bytes((valid[-1] ^ 1,))
    decoder = RtcmDecoder()
    assert decoder.feed(b"noise" + corrupt[:2]) == []
    assert decoder.feed(corrupt[2:] + valid[:3]) == []
    assert decoder.feed(valid[3:]) == [valid]
    assert decoder.crc_errors == 1


def test_http_and_icy_headers_can_be_fragmented() -> None:
    response = b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\nbody"
    assert parse_response(response[:11]) is None
    parsed = parse_response(response)
    assert parsed is not None
    assert parsed.payload == b"body"
    assert parsed.chunked is False
    icy = parse_response(b"ICY 200 OK\r\n" + _frame())
    assert icy is not None
    assert icy.payload == _frame()


def test_http_chunked_decoder_handles_split_chunks_and_extensions() -> None:
    decoder = ChunkDecoder()
    assert decoder.feed(b"3;foo=bar\r\nabc\r") == b""
    assert decoder.feed(b"\n4\r\ndefg\r\n0\r\n\r\n") == b"abcdefg"
    assert decoder.finished is True


@pytest.mark.parametrize(
    "response",
    (
        b"SOURCETABLE 200 OK\r\n",
        b"HTTP/1.1 401 Unauthorized\r\n\r\n",
        b"HTTP/1.1 200 OK\r\nContent-Type: text/plain\r\n\r\n",
        b"HTTP/1.1 200 OK\r\nTransfer-Encoding: gzip\r\n\r\n",
    ),
)
def test_sourcetable_text_and_invalid_responses_are_rejected(response: bytes) -> None:
    with pytest.raises(ConnectionError):
        parse_response(response)
