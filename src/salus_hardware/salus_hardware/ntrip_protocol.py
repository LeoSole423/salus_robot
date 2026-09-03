"""Pure, bounded NTRIP response and RTCM3 stream decoders."""

from __future__ import annotations

from dataclasses import dataclass

from .rtk_domain import crc24q, validate_rtcm3_frame

__all__ = ["ChunkDecoder", "NtripResponse", "RtcmDecoder", "crc24q", "parse_response"]

MAX_HEADER_BYTES = 16 * 1024
MAX_CHUNK_HEADER_BYTES = 1024
MAX_CHUNK_BYTES = 1024 * 1024
MAX_RTCM_FRAME_BYTES = 1029


@dataclass(frozen=True)
class NtripResponse:
    """The parsed response header and bytes already received from its body."""

    payload: bytes
    chunked: bool = False


def parse_response(data: bytes) -> NtripResponse | None:
    """Parse one incrementally received NTRIP response."""

    if not isinstance(data, bytes):
        raise ConnectionError("ntrip_invalid_response")
    if data.startswith(b"SOURCETABLE"):
        raise ConnectionError("ntrip_sourcetable_not_corrections")
    if data.startswith(b"ICY"):
        end = data.find(b"\r\n")
        if end < 0:
            if len(data) > MAX_HEADER_BYTES:
                raise ConnectionError("ntrip_headers_too_large")
            return None
        if data[:end] != b"ICY 200 OK":
            raise ConnectionError("ntrip_rejected")
        return NtripResponse(data[end + 2:])

    end = data.find(b"\r\n\r\n")
    if end < 0:
        if len(data) > MAX_HEADER_BYTES:
            raise ConnectionError("ntrip_headers_too_large")
        return None
    if end > MAX_HEADER_BYTES:
        raise ConnectionError("ntrip_headers_too_large")

    lines = data[:end].decode("latin1").split("\r\n")
    status = lines[0].split()
    if len(status) < 2 or status[0] not in ("HTTP/1.0", "HTTP/1.1"):
        raise ConnectionError("ntrip_invalid_response")
    if status[1] != "200":
        code = status[1] if status[1].isdigit() else "invalid"
        raise ConnectionError(f"ntrip_http_{code}")

    headers: dict[str, str] = {}
    for line in lines[1:]:
        if not line or ":" not in line:
            raise ConnectionError("ntrip_invalid_header")
        key, value = line.split(":", 1)
        headers[key.strip().lower()] = value.strip().lower()

    content_type = headers.get("content-type", "").split(";", 1)[0].strip()
    if "sourcetable" in content_type or content_type.startswith("text/"):
        raise ConnectionError("ntrip_sourcetable_or_text_not_corrections")
    transfer = headers.get("transfer-encoding", "identity")
    if transfer not in ("", "identity", "chunked"):
        raise ConnectionError("ntrip_unsupported_transfer_encoding")
    if headers.get("content-encoding", "identity") not in ("", "identity"):
        raise ConnectionError("ntrip_unsupported_content_encoding")
    return NtripResponse(data[end + 4:], transfer == "chunked")


class ChunkDecoder:
    """Incremental HTTP chunk decoder, including split framing."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.remaining: int | None = None
        self.finished = False

    def feed(self, data: bytes) -> bytes:
        if self.finished:
            return b""
        self.buffer.extend(data)
        result = bytearray()
        while True:
            if self.remaining is None:
                end = self.buffer.find(b"\r\n")
                if end < 0:
                    if len(self.buffer) > MAX_CHUNK_HEADER_BYTES:
                        raise ConnectionError("ntrip_invalid_chunk_header")
                    break
                raw_size = self.buffer[:end].split(b";", 1)[0].strip()
                try:
                    self.remaining = int(raw_size, 16)
                except ValueError:
                    raise ConnectionError("ntrip_invalid_chunk_size") from None
                del self.buffer[: end + 2]
                if not 0 <= self.remaining <= MAX_CHUNK_BYTES:
                    raise ConnectionError("ntrip_chunk_too_large")
                if self.remaining == 0:
                    self.finished = True
                    self.buffer.clear()
                    break
            if len(self.buffer) < self.remaining + 2:
                break
            if self.buffer[self.remaining:self.remaining + 2] != b"\r\n":
                raise ConnectionError("ntrip_invalid_chunk_terminator")
            result.extend(self.buffer[: self.remaining])
            del self.buffer[:self.remaining + 2]
            self.remaining = None
        return bytes(result)


class RtcmDecoder:
    """Resynchronizing incremental RTCM3 decoder with CRC24Q accounting."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes) -> list[bytes]:
        self.buffer.extend(data)
        frames: list[bytes] = []
        while len(self.buffer) >= 6:
            if self.buffer[0] != 0xD3 or self.buffer[1] & 0xFC:
                del self.buffer[0]
                continue
            payload_length = ((self.buffer[1] & 0x03) << 8) | self.buffer[2]
            frame_size = payload_length + 6
            if frame_size > MAX_RTCM_FRAME_BYTES:
                del self.buffer[0]
                continue
            if len(self.buffer) < frame_size:
                break
            frame = bytes(self.buffer[:frame_size])
            if validate_rtcm3_frame(frame) != "accepted":
                self.crc_errors += 1
                del self.buffer[0]
                continue
            del self.buffer[:frame_size]
            frames.append(frame)
        return frames
