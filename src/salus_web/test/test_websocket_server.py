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
        if request.op == "get_nav_snapshot":
            return [{
                "op": "nav_snapshot",
                "ok": True,
                "client_req_id": request.request_id,
                "image_base64": "snapshot",
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


async def _nav_live_scenario() -> None:
    guard = OperatorControlGuard(
        enabled=True,
        heartbeat_timeout_s=2.5,
        initially_locked=True,
        clock=asyncio.get_running_loop().time,
    )
    lease = OperatorLease(guard)
    server = CockpitWebSocketServer(
        FakeGateway(), lease, host="127.0.0.1", port=0
    )
    await server.start()
    port = server._server.sockets[0].getsockname()[1]
    try:
        async with websockets.connect(f"ws://127.0.0.1:{port}") as normal:
            await normal.recv()
            await normal.send(json.dumps({
                "op": "set_control_lock",
                "client_req_id": "normal-unlock",
                "locked": False,
            }))
            await _receive_until(normal, lambda item: item.get("request") == "set_control_lock")
            await _receive_until(normal, lambda item: item.get("op") == "nav_telemetry")

            async with websockets.connect(f"ws://127.0.0.1:{port}/?client=nav-live") as nav_live:
                initial = json.loads(await asyncio.wait_for(nav_live.recv(), 2.0))
                assert initial["op"] == "state"

                await server.broadcast({"op": "scan_preview", "ranges": [1.0]})
                await server.broadcast({"op": "nav_telemetry", "speed_mps": 1.0})
                with pytest.raises(asyncio.TimeoutError):
                    await asyncio.wait_for(nav_live.recv(), 0.1)

                await nav_live.send(json.dumps({
                    "op": "get_nav_snapshot",
                    "client_req_id": "snapshot-1",
                }))
                snapshot = await _receive_until(
                    nav_live, lambda item: item.get("client_req_id") == "snapshot-1"
                )
                assert snapshot["op"] == "nav_snapshot"
                assert snapshot["ok"] is True

                await nav_live.send(json.dumps({
                    "op": "set_control_lock",
                    "client_req_id": "nav-lock",
                    "locked": False,
                }))
                rejected = await _receive_until(
                    nav_live, lambda item: item.get("client_req_id") == "nav-lock"
                )
                assert rejected["error_code"] == "NAV_LIVE_READ_ONLY"
                assert lease.state_for("__test__").owner_present is True

                await server.broadcast({"op": "scan_preview", "ranges": [2.0]})
                preview = await _receive_until(normal, lambda item: item.get("op") == "scan_preview")
                assert preview["ranges"] == [2.0]
    finally:
        await server.stop()


def test_nav_live_is_read_only_and_does_not_receive_unsolicited_broadcasts() -> None:
    asyncio.run(asyncio.wait_for(_nav_live_scenario(), 8.0))
