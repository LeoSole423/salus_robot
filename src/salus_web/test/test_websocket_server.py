import asyncio
import json

import pytest
import websockets

from salus_web.operator_guard import OperatorControlGuard
from salus_web.operator_lease import OperatorLease
from salus_web.protocol import ack
from salus_web.websocket_server import (
    ClientOutbox,
    CockpitWebSocketServer,
    SlowClientError,
    _json_safe,
)


class FakeGateway:
    async def initial_state(self):
        return {
            "op": "state", "ok": True, "connected": True,
            "control_locked": True, "control_lock_reason": "STARTUP_LOCKED",
        }

    async def dispatch(self, request):
        if request.op == "get_state":
            return [{
                "op": "state",
                "ok": True,
                "connected": True,
                "client_req_id": request.request_id,
                "control_locked": True,
                "control_lock_reason": "STARTUP_LOCKED",
            }]
        return [ack(request, ok=True)]


async def _outbox_scenario() -> None:
    outbox = ClientOutbox(2)
    await outbox.put({"op": "state", "value": 1})
    await outbox.put({"op": "state", "value": 2})
    assert await outbox.get() == {"op": "state", "value": 2}
    await outbox.put({"op": "ack", "request": "one"})
    await outbox.put({"op": "ack", "request": "two"})
    with pytest.raises(SlowClientError):
        await outbox.put({"op": "ack", "request": "three"})


def test_outbox_coalesces_state_and_protects_acknowledgements() -> None:
    asyncio.run(_outbox_scenario())


def test_non_finite_ros_values_are_degraded_before_json_encoding() -> None:
    assert _json_safe({"valid": 1.0, "missing": float("nan")}) == {
        "valid": 1.0,
        "missing": None,
    }


async def _receive_until(socket, predicate, limit=8):
    for _ in range(limit):
        message = json.loads(await asyncio.wait_for(socket.recv(), 2.0))
        if predicate(message):
            return message
    raise AssertionError("expected WebSocket message was not received")


async def _server_scenario() -> None:
    guard = OperatorControlGuard(
        enabled=True,
        heartbeat_timeout_s=2.5,
        initially_locked=True,
        clock=asyncio.get_running_loop().time,
    )
    server = CockpitWebSocketServer(
        FakeGateway(), OperatorLease(guard), host="127.0.0.1", port=0
    )
    await server.start()
    port = server._server.sockets[0].getsockname()[1]
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as first:
            initial = json.loads(await asyncio.wait_for(first.recv(), 2.0))
            assert initial["control_locked"] is True

            await first.send("{")
            invalid = json.loads(await asyncio.wait_for(first.recv(), 2.0))
            assert invalid["error_code"] == "invalid_json"

            await first.send(json.dumps({
                "op": "set_control_lock",
                "client_req_id": "unlock-1",
                "locked": False,
            }))
            unlocked = await _receive_until(
                first, lambda item: item.get("request") == "set_control_lock"
            )
            assert unlocked["ok"] is True
            assert unlocked["control_owner"] is True
            unlocked_telemetry = await _receive_until(
                first,
                lambda item: item.get("op") == "nav_telemetry"
                and item.get("control_locked") is False,
            )
            assert unlocked_telemetry["control_owner"] is True

            await first.send(json.dumps({
                "op": "get_state",
                "client_req_id": "state-after-unlock",
            }))
            unlocked_state = await _receive_until(
                first, lambda item: item.get("client_req_id") == "state-after-unlock"
            )
            assert unlocked_state["control_locked"] is False
            assert unlocked_state["control_lock_reason"] == ""
            assert unlocked_state["control_owner"] is True

            async with websockets.connect(f"ws://127.0.0.1:{port}") as second:
                second_initial = json.loads(await asyncio.wait_for(second.recv(), 2.0))
                assert second_initial["control_owner_present"] is True
                assert second_initial["control_owner"] is False
                await second.send(json.dumps({
                    "op": "set_goal_ll",
                    "client_req_id": "goal-2",
                    "waypoints": [{"lat": -31.0, "lon": -64.0}],
                }))
                rejected = await _receive_until(
                    second, lambda item: item.get("client_req_id") == "goal-2"
                )
                assert rejected["error_code"] == "CONTROL_OWNED"
    finally:
        await server.stop()


def test_real_websocket_transport_correlates_and_enforces_lease() -> None:
    asyncio.run(asyncio.wait_for(_server_scenario(), 8.0))
