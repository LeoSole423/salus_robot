import socketserver
import threading
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter

from salus_hardware.ntrip_rtcm_source import NtripRtcmSourceNode
from salus_hardware.rtk_domain import crc24q


class CapturingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _frame(payload: bytes = b"\x3e\x00") -> bytes:
    body = b"\xd3" + bytes((0, len(payload))) + payload
    return body + crc24q(body).to_bytes(3, "big")


class _FakeCaster(socketserver.ThreadingTCPServer):
    allow_reuse_address = True


class _FakeCasterHandler(socketserver.BaseRequestHandler):
    request_seen = threading.Event()
    release = threading.Event()
    frame = _frame()
    corrupt_first = False
    send_frame_once = False
    connection_count = 0
    last_request = b""

    def handle(self):
        self.request.settimeout(1.0)
        data = bytearray()
        while b"\r\n\r\n" not in data:
            chunk = self.request.recv(4096)
            if not chunk:
                return
            data.extend(chunk)
        type(self).last_request = bytes(data)
        type(self).connection_count += 1
        type(self).request_seen.set()
        response = b"HTTP/1.1 200 OK\r\nContent-Type: gnss/data\r\n\r\n"
        self.request.sendall(response)
        if not type(self).send_frame_once or type(self).connection_count == 1:
            frame = type(self).frame
            if type(self).corrupt_first:
                frame = frame[:-1] + bytes((frame[-1] ^ 1,))
            self.request.sendall(frame)
        type(self).release.wait(timeout=3.0)


def _write_config(path: Path, port: int) -> None:
    path.write_text(
        "active_source_id: fake\n"
        "sources:\n"
        "  - id: fake\n"
        "    label: Fake caster\n"
        "    host: 127.0.0.1\n"
        f"    port: {port}\n"
        "    mountpoint: RTCM3\n"
        "    username: test-user\n"
        "    password: test-password\n",
        encoding="utf-8",
    )


def test_fake_caster_publishes_one_canonical_frame_and_sanitized_status(tmp_path: Path) -> None:
    _FakeCasterHandler.request_seen.clear()
    _FakeCasterHandler.release.clear()
    _FakeCasterHandler.corrupt_first = False
    _FakeCasterHandler.send_frame_once = False
    _FakeCasterHandler.connection_count = 0
    server = _FakeCaster(("127.0.0.1", 0), _FakeCasterHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    config = tmp_path / "sources.yaml"
    _write_config(config, server.server_address[1])

    rclpy.init()
    node = NtripRtcmSourceNode(
        parameter_overrides=[
            Parameter("sources_config", value=str(config)),
            Parameter("status_period_s", value=10.0),
            Parameter("reconnect_delay_s", value=0.1),
            Parameter("max_reconnect_delay_s", value=0.1),
        ]
    )
    rtcm_pub = CapturingPublisher()
    status_pub = CapturingPublisher()
    node._rtcm_pub = rtcm_pub
    node._status_pub = status_pub
    try:
        assert _FakeCasterHandler.request_seen.wait(timeout=2.0)
        assert _FakeCasterHandler.last_request.startswith(b"GET /RTCM3 HTTP/1.1")
        assert b"Authorization: Basic " in _FakeCasterHandler.last_request
        assert b"User-Agent: NTRIP RTKLIB/2.4.3\r\n" in _FakeCasterHandler.last_request
        deadline = time.monotonic() + 2.0
        while len(rtcm_pub.messages) < 1 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(rtcm_pub.messages) == 1
        message = rtcm_pub.messages[0]
        assert message.source_id == "fake"
        assert message.sequence == 1
        assert bytes(message.data) == _FakeCasterHandler.frame
        node._publish_status()
        status = status_pub.messages[-1]
        assert status.fix_quality == status.UNKNOWN
        assert status.delivery_backend == status.BACKEND_DISABLED
        assert status.delivery_state == status.DELIVERY_DISABLED
        assert status.acquisition_state == status.ACQUISITION_RECEIVING
        assert status.status_detail == "receiving_rtcm"
        assert "test-password" not in status.status_detail
    finally:
        _FakeCasterHandler.release.set()
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()


def test_stale_corrections_are_reported_before_reconnect(tmp_path: Path) -> None:
    _FakeCasterHandler.request_seen.clear()
    _FakeCasterHandler.release.clear()
    _FakeCasterHandler.corrupt_first = False
    _FakeCasterHandler.send_frame_once = True
    _FakeCasterHandler.connection_count = 0
    server = _FakeCaster(("127.0.0.1", 0), _FakeCasterHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    config = tmp_path / "sources.yaml"
    _write_config(config, server.server_address[1])
    rclpy.init()
    node = NtripRtcmSourceNode(
        parameter_overrides=[
            Parameter("sources_config", value=str(config)),
            Parameter("status_period_s", value=10.0),
            Parameter("read_timeout_s", value=0.05),
            Parameter("reconnect_delay_s", value=0.1),
            Parameter("max_reconnect_delay_s", value=0.1),
            Parameter("rtcm_stale_timeout_s", value=0.1),
        ]
    )
    rtcm_pub = CapturingPublisher()
    status_pub = CapturingPublisher()
    node._rtcm_pub = rtcm_pub
    node._status_pub = status_pub
    saw_stale = False
    saw_reconnect = False
    try:
        assert _FakeCasterHandler.request_seen.wait(timeout=2.0)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            node._publish_status()
            saw_stale = saw_stale or (
                status_pub.messages[-1].acquisition_state
                == status_pub.messages[-1].ACQUISITION_STALE
            )
            saw_reconnect = _FakeCasterHandler.connection_count >= 2
            if saw_stale and saw_reconnect:
                break
            time.sleep(0.01)
        assert len(rtcm_pub.messages) == 1
        assert saw_stale is True
        assert saw_reconnect is True
    finally:
        _FakeCasterHandler.release.set()
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()
        _FakeCasterHandler.send_frame_once = False


def test_corrupt_frame_increments_crc_without_publication(tmp_path: Path) -> None:
    _FakeCasterHandler.request_seen.clear()
    _FakeCasterHandler.release.clear()
    _FakeCasterHandler.corrupt_first = True
    _FakeCasterHandler.send_frame_once = False
    _FakeCasterHandler.connection_count = 0
    server = _FakeCaster(("127.0.0.1", 0), _FakeCasterHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    config = tmp_path / "sources.yaml"
    _write_config(config, server.server_address[1])
    rclpy.init()
    node = NtripRtcmSourceNode(
        parameter_overrides=[
            Parameter("sources_config", value=str(config)),
            Parameter("status_period_s", value=10.0),
            Parameter("reconnect_delay_s", value=0.1),
            Parameter("max_reconnect_delay_s", value=0.1),
        ]
    )
    rtcm_pub = CapturingPublisher()
    node._rtcm_pub = rtcm_pub
    try:
        assert _FakeCasterHandler.request_seen.wait(timeout=2.0)
        time.sleep(0.2)
        assert rtcm_pub.messages == []
        assert node._crc_error_count >= 1
    finally:
        _FakeCasterHandler.release.set()
        node.destroy_node()
        rclpy.shutdown()
        server.shutdown()
        server.server_close()
        _FakeCasterHandler.corrupt_first = False
