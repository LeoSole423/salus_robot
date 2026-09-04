"""NTRIP caster acquisition node publishing canonical RTCM frames."""

from __future__ import annotations

import base64
import math
import socket
import threading
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from salus_interfaces.msg import GnssRtkStatus, RtcmFrame

from .ntrip_protocol import ChunkDecoder, NtripResponse, RtcmDecoder, parse_response
from .ntrip_source_config import NtripSource, load_sources, validate_positive_finite

RTCM_BUFFER_SIZE = 4096


class NtripRtcmSourceNode(Node):
    """Own exactly one read-only NTRIP source selected at startup."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__("ntrip_rtcm_source", parameter_overrides=parameter_overrides)
        defaults = {
            "sources_config": "",
            "active_source_id": "",
            "rtcm_topic": "/salus/hardware/rtcm/corrections",
            "status_topic": "/salus/hardware/gnss_primary/rtk_source_status",
            "status_period_s": 1.0,
            "connect_timeout_s": 5.0,
            "read_timeout_s": 2.0,
            "reconnect_delay_s": 2.0,
            "max_reconnect_delay_s": 60.0,
            "rtcm_stale_timeout_s": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        config_value = str(self.get_parameter("sources_config").value).strip()
        if not config_value:
            raise ValueError("sources_config is required")
        self._status_period_s = validate_positive_finite(
            self.get_parameter("status_period_s").value, "status_period_s"
        )
        self._connect_timeout_s = validate_positive_finite(
            self.get_parameter("connect_timeout_s").value, "connect_timeout_s"
        )
        self._read_timeout_s = validate_positive_finite(
            self.get_parameter("read_timeout_s").value, "read_timeout_s"
        )
        self._reconnect_delay_s = validate_positive_finite(
            self.get_parameter("reconnect_delay_s").value, "reconnect_delay_s"
        )
        self._max_reconnect_delay_s = validate_positive_finite(
            self.get_parameter("max_reconnect_delay_s").value,
            "max_reconnect_delay_s",
        )
        self._rtcm_stale_timeout_s = validate_positive_finite(
            self.get_parameter("rtcm_stale_timeout_s").value,
            "rtcm_stale_timeout_s",
        )
        if self._max_reconnect_delay_s < self._reconnect_delay_s:
            raise ValueError("max_reconnect_delay_s must not be below reconnect_delay_s")

        sources, configured_active_id = load_sources(Path(config_value))
        by_id = {source.id: source for source in sources}
        override_active_id = str(
            self.get_parameter("active_source_id").value
        ).strip()
        active_id = override_active_id or configured_active_id
        if not active_id:
            raise ValueError("active_source_id is required in config or launch")
        if active_id not in by_id:
            raise ValueError("unknown_active_source_id")

        self._source = by_id[active_id]
        self._rtcm_topic = str(self.get_parameter("rtcm_topic").value).strip()
        self._status_topic = str(self.get_parameter("status_topic").value).strip()
        if not self._rtcm_topic or not self._status_topic:
            raise ValueError("RTCM and status topics are required")

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._socket: socket.socket | None = None
        self._connected = False
        self._state = GnssRtkStatus.ACQUISITION_DISCONNECTED
        self._last_error = ""
        self._last_frame_monotonic: float | None = None
        self._received_count = 0
        self._crc_error_count = 0
        self._sequence = 0

        self._rtcm_pub = self.create_publisher(RtcmFrame, self._rtcm_topic, 10)
        self._status_pub = self.create_publisher(GnssRtkStatus, self._status_topic, 10)
        self.create_timer(self._status_period_s, self._publish_status)
        self._worker = threading.Thread(
            target=self._reader_loop, name="ntrip_rtcm_source", daemon=True
        )
        self._worker.start()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._wake_event.set()
        self._close_socket()
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        return super().destroy_node()

    def _reader_loop(self) -> None:
        delay = self._reconnect_delay_s
        while not self._stop_event.is_set():
            try:
                sock, response = self._open_stream(self._source)
                self._set_connected(True, "")
                decoder = RtcmDecoder()
                chunks = ChunkDecoder() if response.chunked else None
                last_valid: float | None = None
                payload = response.payload
                while not self._stop_event.is_set():
                    body = chunks.feed(payload) if chunks else payload
                    decoder_frames = decoder.feed(body)
                    self._add_crc_errors(decoder.crc_errors)
                    decoder.crc_errors = 0
                    for frame in decoder_frames:
                        self._publish_frame(frame)
                        last_valid = time.monotonic()
                        delay = self._reconnect_delay_s
                    if chunks and chunks.finished:
                        raise ConnectionError("ntrip_stream_closed")
                    if (
                        last_valid is not None
                        and time.monotonic() - last_valid > self._rtcm_stale_timeout_s
                    ):
                        self._set_stale()
                        raise ConnectionError("ntrip_corrections_stale")
                    try:
                        payload = sock.recv(RTCM_BUFFER_SIZE)
                    except socket.timeout:
                        payload = b""
                        continue
                    if not payload:
                        raise ConnectionError("ntrip_stream_closed")
            except InterruptedError:
                pass
            except ConnectionError as error:
                if str(error) != "ntrip_corrections_stale":
                    self._set_error(str(error))
            except (OSError, ValueError):
                self._set_error("ntrip_connection_failed")
            finally:
                self._close_socket()
            if self._wake_event.wait(timeout=delay):
                self._wake_event.clear()
                delay = self._reconnect_delay_s
            else:
                delay = min(delay * 2.0, self._max_reconnect_delay_s)

    def _open_stream(self, source: NtripSource) -> tuple[socket.socket, NtripResponse]:
        sock = socket.create_connection(
            (source.host, source.port), timeout=self._connect_timeout_s
        )
        with self._lock:
            if self._stop_event.is_set():
                sock.close()
                raise InterruptedError
            self._socket = sock
        sock.settimeout(self._read_timeout_s)
        credentials = f"{source.username}:{source.password}".encode("utf-8")
        auth = base64.b64encode(credentials).decode("ascii")
        host_header = f"[{source.host}]" if ":" in source.host else source.host
        request = (
            f"GET /{source.mountpoint.lstrip('/')} HTTP/1.1\r\n"
            f"Host: {host_header}:{source.port}\r\n"
            "User-Agent: NTRIP RTKLIB/2.4.3\r\n"
            "Accept: */*\r\n"
            "Connection: close\r\n"
            "Ntrip-Version: Ntrip/2.0\r\n"
            f"Authorization: Basic {auth}\r\n"
            "\r\n"
        )
        sock.sendall(request.encode("ascii"))

        response_data = bytearray()
        deadline = time.monotonic() + self._connect_timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ConnectionError("ntrip_handshake_timeout")
            sock.settimeout(min(self._read_timeout_s, remaining))
            try:
                chunk = sock.recv(RTCM_BUFFER_SIZE)
            except socket.timeout:
                continue
            if not chunk:
                raise ConnectionError("ntrip_closed_during_handshake")
            response_data.extend(chunk)
            parsed = parse_response(bytes(response_data))
            if parsed is not None:
                sock.settimeout(self._read_timeout_s)
                return sock, parsed

    def _publish_frame(self, frame: bytes) -> None:
        message = RtcmFrame()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = ""
        message.source_id = self._source.id
        with self._lock:
            if self._stop_event.is_set():
                return
            self._sequence += 1
            message.sequence = self._sequence
            message.data = list(frame)
            self._rtcm_pub.publish(message)
            self._last_frame_monotonic = time.monotonic()
            self._received_count += 1
            self._connected = True
            self._state = GnssRtkStatus.ACQUISITION_RECEIVING
            self._last_error = ""

    def _publish_status(self) -> None:
        now = time.monotonic()
        with self._lock:
            age = (
                max(0.0, now - self._last_frame_monotonic)
                if self._last_frame_monotonic is not None
                else math.nan
            )
            state = self._state
            if self._connected and self._last_frame_monotonic is None:
                state = GnssRtkStatus.ACQUISITION_CONNECTED_NO_DATA
            elif (
                self._connected
                and self._last_frame_monotonic is not None
                and age > self._rtcm_stale_timeout_s
            ):
                state = GnssRtkStatus.ACQUISITION_STALE
                self._state = state
            detail = self._status_detail(state)
            message = GnssRtkStatus()
            message.header.stamp = self.get_clock().now().to_msg()
            message.fix_quality = GnssRtkStatus.UNKNOWN
            message.acquisition_state = state
            message.delivery_backend = GnssRtkStatus.BACKEND_DISABLED
            message.delivery_state = GnssRtkStatus.DELIVERY_DISABLED
            message.receiver_fix_type = -1
            message.satellites_visible = 255
            message.corrections_fresh = (
                state == GnssRtkStatus.ACQUISITION_RECEIVING
                and math.isfinite(age)
                and age <= self._rtcm_stale_timeout_s
            )
            message.correction_age_s = float(age)
            message.received_count = self._received_count
            message.crc_error_count = self._crc_error_count
            message.source_id = self._source.id
            message.status_detail = detail
        self._status_pub.publish(message)

    def _status_detail(self, state: int) -> str:
        if state == GnssRtkStatus.ACQUISITION_ERROR:
            return self._last_error or "ntrip_error"
        return {
            GnssRtkStatus.ACQUISITION_DISCONNECTED: "disconnected",
            GnssRtkStatus.ACQUISITION_CONNECTED_NO_DATA: "connected_no_data",
            GnssRtkStatus.ACQUISITION_RECEIVING: "receiving_rtcm",
            GnssRtkStatus.ACQUISITION_STALE: "corrections_stale",
        }.get(state, "disconnected")

    def _set_connected(self, connected: bool, error: str) -> None:
        with self._lock:
            self._connected = connected
            self._last_error = error
            if connected:
                self._last_frame_monotonic = None
                self._state = GnssRtkStatus.ACQUISITION_CONNECTED_NO_DATA
            else:
                self._state = GnssRtkStatus.ACQUISITION_DISCONNECTED

    def _set_stale(self) -> None:
        with self._lock:
            self._connected = False
            self._state = GnssRtkStatus.ACQUISITION_STALE
            self._last_error = ""

    def _set_error(self, error: str) -> None:
        safe_error = error if error.startswith("ntrip_") else "ntrip_connection_failed"
        with self._lock:
            self._connected = False
            self._state = GnssRtkStatus.ACQUISITION_ERROR
            self._last_error = safe_error

    def _add_crc_errors(self, count: int) -> None:
        if count:
            with self._lock:
                self._crc_error_count += count

    def _close_socket(self) -> None:
        with self._lock:
            sock = self._socket
            self._socket = None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NtripRtcmSourceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
