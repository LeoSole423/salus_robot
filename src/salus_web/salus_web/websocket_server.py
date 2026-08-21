"""Async WebSocket transport isolated from ROS message types."""

from __future__ import annotations

import asyncio
from collections import deque
from dataclasses import dataclass
import json
import logging
import math
from typing import Any, Iterable, Mapping, Protocol
import uuid

import websockets

from .operator_lease import OperatorLease
from .protocol import ProtocolError, ack, parse_request, validate_request


REPLACEABLE_OPS = frozenset(
    {"state", "nav_telemetry", "robot_pose", "gps_status", "drive_telemetry", "sensor_info"}
)


class Gateway(Protocol):
    async def dispatch(self, request: Any) -> Iterable[dict[str, Any]]:
        ...

    async def initial_state(self) -> dict[str, Any]:
        ...


class SlowClientError(RuntimeError):
    pass


@dataclass(frozen=True)
class OutgoingMessage:
    payload: dict[str, Any]
    replaceable: bool


class ClientOutbox:
    """Bounded queue that coalesces state but never silently drops acknowledgements."""

    def __init__(self, capacity: int) -> None:
        if capacity < 2:
            raise ValueError("capacity must be at least two")
        self._capacity = capacity
        self._items: deque[OutgoingMessage] = deque()
        self._ready = asyncio.Condition()
        self._closed = False

    async def put(self, payload: dict[str, Any]) -> None:
        replaceable = str(payload.get("op", "")) in REPLACEABLE_OPS
        async with self._ready:
            if self._closed:
                return
            if replaceable:
                operation = payload.get("op")
                for index, item in enumerate(self._items):
                    if item.replaceable and item.payload.get("op") == operation:
                        self._items[index] = OutgoingMessage(payload, True)
                        self._ready.notify()
                        return
            if len(self._items) >= self._capacity:
                if not replaceable:
                    raise SlowClientError("non-replaceable WebSocket queue overflow")
                removable = next(
                    (index for index, item in enumerate(self._items) if item.replaceable),
                    None,
                )
                if removable is None:
                    return
                del self._items[removable]
            self._items.append(OutgoingMessage(payload, replaceable))
            self._ready.notify()

    async def get(self) -> dict[str, Any] | None:
        async with self._ready:
            await self._ready.wait_for(lambda: self._items or self._closed)
            if not self._items:
                return None
            return self._items.popleft().payload

    async def close(self) -> None:
        async with self._ready:
            self._closed = True
            self._ready.notify_all()


@dataclass
class _Client:
    websocket: Any
    outbox: ClientOutbox
    writer: asyncio.Task[Any]


class CockpitWebSocketServer:
    def __init__(
        self,
        gateway: Gateway,
        lease: OperatorLease,
        *,
        host: str = "0.0.0.0",
        port: int = 8766,
        queue_capacity: int = 64,
    ) -> None:
        self._gateway = gateway
        self._lease = lease
        self._host = host
        self._port = port
        self._queue_capacity = queue_capacity
        self._clients: dict[str, _Client] = {}
        self._server = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._watchdog: asyncio.Task[Any] | None = None
        self._logger = logging.getLogger("salus_web.websocket")

    async def start(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._server = await websockets.serve(self._connection, self._host, self._port)
        self._watchdog = asyncio.create_task(self._heartbeat_watchdog())

    async def stop(self) -> None:
        if self._watchdog is not None:
            self._watchdog.cancel()
            await asyncio.gather(self._watchdog, return_exceptions=True)
            self._watchdog = None
        if self._server is not None:
            # Stop accepting clients, then close every active socket.  Closing
            # only writer tasks leaves the server handler blocked in its
            # receive loop and makes wait_closed() wait forever.
            self._server.close()
            clients = list(self._clients.values())
            if clients:
                await asyncio.gather(
                    *(client.websocket.close(code=1001, reason="server shutdown")
                      for client in clients),
                    return_exceptions=True,
                )
            await self._server.wait_closed()
            self._server = None
        # Connection handlers normally own this cleanup.  The fallback keeps
        # shutdown bounded if a transport disappeared without running it.
        clients = list(self._clients.values())
        for client in clients:
            await client.outbox.close()
            client.writer.cancel()
        if clients:
            await asyncio.gather(
                *(client.writer for client in clients), return_exceptions=True
            )
        self._clients.clear()

    def broadcast_from_thread(self, payload: dict[str, Any]) -> None:
        if self._loop is None or self._loop.is_closed():
            return
        self._loop.call_soon_threadsafe(
            lambda: asyncio.create_task(self.broadcast(payload))
        )

    async def broadcast(self, payload: dict[str, Any]) -> None:
        for client_id, client in list(self._clients.items()):
            try:
                await client.outbox.put(self._decorate(payload, client_id))
            except SlowClientError:
                await client.websocket.close(code=1013, reason="operator client too slow")
                self._logger.warning("closed slow Cockpit client %s", client_id)

    async def _connection(self, websocket: Any, _path: str | None = None) -> None:
        client_id = uuid.uuid4().hex
        outbox = ClientOutbox(self._queue_capacity)
        writer = asyncio.create_task(self._writer(websocket, outbox))
        self._clients[client_id] = _Client(websocket, outbox, writer)
        pending: set[asyncio.Task[Any]] = set()
        try:
            await outbox.put(self._decorate(await self._gateway.initial_state(), client_id))
            async for raw in websocket:
                task = asyncio.create_task(
                    self._handle_safe(client_id, websocket, outbox, raw)
                )
                pending.add(task)
                task.add_done_callback(pending.discard)
        finally:
            if pending:
                await asyncio.gather(*pending, return_exceptions=True)
            lease_changed = self._lease.disconnect(client_id)
            self._clients.pop(client_id, None)
            await outbox.close()
            writer.cancel()
            await asyncio.gather(writer, return_exceptions=True)
            if lease_changed:
                await self.broadcast(await self._gateway.initial_state())

    async def _handle_safe(
        self, client_id: str, websocket: Any, outbox: ClientOutbox, raw: Any
    ) -> None:
        try:
            await self._handle(client_id, outbox, raw)
        except SlowClientError:
            await websocket.close(code=1013, reason="operator client too slow")

    async def _handle(self, client_id: str, outbox: ClientOutbox, raw: Any) -> None:
        try:
            request = validate_request(parse_request(raw))
        except ProtocolError as error:
            await outbox.put(ack(
                error,
                ok=False,
                error=error.message,
                error_code=error.code,
            ))
            return

        if request.op == "set_control_lock":
            decision = self._lease.set_locked(client_id, request.fields["locked"])
            await outbox.put(self._lease_ack(request, decision))
            if decision.allowed:
                await self.broadcast(await self._gateway.initial_state())
            return
        if request.op == "control_heartbeat":
            decision = self._lease.heartbeat(client_id)
            await outbox.put(self._lease_ack(request, decision))
            return

        decision = self._lease.authorize(client_id, request)
        if not decision.allowed:
            await outbox.put(self._lease_ack(request, decision))
            return
        try:
            for response in await self._gateway.dispatch(request):
                await outbox.put(response)
        except Exception as exc:
            self._logger.exception("Cockpit operation %s failed", request.op)
            await outbox.put(ack(
                request,
                ok=False,
                error=str(exc),
                error_code="GATEWAY_FAILURE",
            ))

    async def _writer(self, websocket: Any, outbox: ClientOutbox) -> None:
        while True:
            payload = await outbox.get()
            if payload is None:
                return
            await websocket.send(
                json.dumps(_json_safe(payload), separators=(",", ":"), allow_nan=False)
            )

    async def _heartbeat_watchdog(self) -> None:
        previous = self._lock_signature()
        while True:
            await asyncio.sleep(0.25)
            current = self._lock_signature()
            if current != previous and current[0] and current[1] == "UI_HEARTBEAT_TIMEOUT":
                await self.broadcast(await self._gateway.initial_state())
            previous = current

    def _lock_signature(self) -> tuple[bool, str, bool]:
        state = self._lease.state_for("__watchdog__")
        return state.lock.locked, state.lock.reason, state.owner_present

    def _decorate(self, payload: dict[str, Any], client_id: str) -> dict[str, Any]:
        output = dict(payload)
        if output.get("op") not in {"state", "nav_telemetry"}:
            return output
        state = self._lease.state_for(client_id)
        output.update({
            "control_locked": state.lock.locked,
            "control_lock_reason": state.lock.reason,
            "locked": state.lock.locked,
            "lock_reason": state.lock.reason,
            "control_owner_present": state.owner_present,
            "control_owner": state.requester_is_owner,
        })
        return output

    @staticmethod
    def _lease_ack(request: Any, decision: Any) -> dict[str, Any]:
        state = decision.state
        return ack(
            request,
            ok=decision.allowed,
            error=None if decision.allowed else decision.error_code,
            error_code=decision.error_code,
            control_locked=state.lock.locked,
            control_lock_reason=state.lock.reason,
            control_owner_present=state.owner_present,
            control_owner=state.requester_is_owner,
        )


def _json_safe(value: Any) -> Any:
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value
