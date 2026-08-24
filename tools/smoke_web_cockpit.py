#!/usr/bin/env python3
"""Exercise the real Cockpit WebSocket boundary against the integrated sim."""

import asyncio
import json
import os
from pathlib import Path
import sys
import time

import websockets


class CockpitProbe:
    def __init__(self, uri: str) -> None:
        self.uri = uri
        self.socket = None
        self.broadcasts = []
        self.requests = 0

    async def connect(self, timeout_s: float = 45.0) -> dict:
        deadline = time.monotonic() + timeout_s
        error = None
        while time.monotonic() < deadline:
            try:
                self.socket = await websockets.connect(self.uri)
                state_deadline = min(deadline, time.monotonic() + 8.0)
                while time.monotonic() < state_deadline:
                    incoming = json.loads(await asyncio.wait_for(
                        self.socket.recv(), state_deadline - time.monotonic()
                    ))
                    if incoming.get("op") == "state":
                        return incoming
                    self.broadcasts.append(incoming)
                raise RuntimeError("initial state frame did not arrive")
            except Exception as exc:
                error = exc
                if self.socket is not None:
                    await self.socket.close()
                    self.socket = None
                await asyncio.sleep(0.25)
        raise RuntimeError(f"WebSocket did not become ready: {error}")

    async def request(self, operation: str, payload=None, timeout_s: float = 10.0):
        self.requests += 1
        request_id = f"smoke-{self.requests}"
        message = {"op": operation, "client_req_id": request_id}
        if payload:
            message.update(payload)
        await self.socket.send(json.dumps(message))
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            incoming = json.loads(
                await asyncio.wait_for(self.socket.recv(), deadline - time.monotonic())
            )
            if incoming.get("client_req_id") == request_id:
                return incoming
            self.broadcasts.append(incoming)
        raise RuntimeError(f"{operation} response timed out")

    async def receive_until(self, predicate, timeout_s: float = 8.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            incoming = json.loads(await asyncio.wait_for(
                self.socket.recv(), deadline - time.monotonic()
            ))
            if predicate(incoming):
                return incoming
            self.broadcasts.append(incoming)
        raise RuntimeError("expected WebSocket message timed out")

    async def close(self):
        if self.socket is not None:
            await self.socket.close()

    async def heartbeat_loop(self, interval_s: float = 0.75) -> None:
        """Keep the lease alive without competing for socket reads.

        Heartbeat acknowledgements remain in the single receive stream and are
        drained by ``request`` like any other asynchronous bridge message.
        """
        sequence = 0
        while True:
            await asyncio.sleep(interval_s)
            sequence += 1
            await self.socket.send(json.dumps({
                "op": "control_heartbeat",
                "client_req_id": f"heartbeat-{sequence}",
            }))


def require_ok(response, operation):
    if response.get("ok") is not True:
        raise RuntimeError(f"{operation} failed: {response}")


async def scenario() -> dict:
    port = int(os.environ.get("SALUS_WEB_SMOKE_PORT", "18766"))
    first = CockpitProbe(f"ws://127.0.0.1:{port}")
    second = CockpitProbe(f"ws://127.0.0.1:{port}")
    heartbeat_task = None
    evidence = {"port": port, "operations": [], "broadcast_ops": []}
    try:
        initial = await first.connect()
        if initial.get("op") != "state" or initial.get("control_locked") is not True:
            raise RuntimeError(f"invalid initial state: {initial}")

        camera_state = await first.request("get_camera_ptz_state")
        require_ok(camera_state, "get_camera_ptz_state")
        if camera_state.get("payload", {}).get("ok") is not True:
            raise RuntimeError(f"sim camera is unavailable: {camera_state}")
        locked_camera = await first.request(
            "camera_ptz_move", {"relative": True, "pan_deg": 5.0}
        )
        if locked_camera.get("error_code") != "CONTROL_LOCKED":
            raise RuntimeError(
                f"locked camera mutation was not rejected: {locked_camera}"
            )

        preview = await first.receive_until(
            lambda item: item.get("op") == "scan_preview", 15.0
        )
        if preview.get("frame_id") != "base_footprint":
            raise RuntimeError(f"scan preview frame is invalid: {preview}")
        if not isinstance(preview.get("ranges"), list) or not preview["ranges"]:
            raise RuntimeError("scan preview omitted reduced ranges")
        if not isinstance(preview.get("valid_count"), int):
            raise RuntimeError("scan preview omitted valid_count")
        evidence["scan_preview"] = {
            "frame_id": preview["frame_id"],
            "range_count": len(preview["ranges"]),
            "valid_count": preview["valid_count"],
        }

        await first.socket.send("{")
        invalid = await first.receive_until(
            lambda item: item.get("error_code") == "invalid_json", 3.0
        )
        if invalid.get("error_code") != "invalid_json":
            raise RuntimeError(f"invalid JSON was not rejected: {invalid}")

        unlocked = await first.request("set_control_lock", {"locked": False})
        require_ok(unlocked, "set_control_lock")
        if unlocked.get("control_owner") is not True:
            raise RuntimeError("unlocking connection did not own the lease")
        evidence["operations"].append("unlock")
        heartbeat_task = asyncio.create_task(first.heartbeat_loop())

        second_initial = await second.connect()
        if second_initial.get("control_owner_present") is not True:
            raise RuntimeError("second client did not observe the active lease")
        rejected = await second.request(
            "set_goal_ll",
            {"waypoints": [{"lat": -31.0, "lon": -64.0}]},
        )
        if rejected.get("error_code") != "CONTROL_OWNED":
            raise RuntimeError(f"second client was not rejected: {rejected}")
        rejected_camera = await second.request(
            "camera_ptz_preset", {"preset": "left"}
        )
        if rejected_camera.get("error_code") != "CONTROL_OWNED":
            raise RuntimeError(
                f"second camera client was not rejected: {rejected_camera}"
            )
        evidence["operations"].append("exclusive_lease")

        moved = await first.request(
            "camera_ptz_move",
            {
                "relative": False,
                "pan_deg": 25.0,
                "tilt_deg": 10.0,
                "zoom_level": 2.0,
            },
        )
        require_ok(moved, "camera_ptz_move")
        moved_state = moved.get("payload", {})
        if abs(float(moved_state.get("pan_deg", -1.0)) - 25.0) > 0.01:
            raise RuntimeError(f"camera absolute move was not applied: {moved}")
        saved = await first.request(
            "camera_ptz_set_preset",
            {"preset": "home", "save_zoom": True},
        )
        require_ok(saved, "camera_ptz_set_preset")
        preset = await first.request(
            "camera_ptz_preset", {"preset": "left"}
        )
        require_ok(preset, "camera_ptz_preset")
        if preset.get("payload", {}).get("active_preset") != "left":
            raise RuntimeError(f"camera preset was not reflected: {preset}")
        evidence["camera"] = {
            "initial": camera_state.get("payload"),
            "moved": moved_state,
            "preset": preset.get("payload"),
            "saved": saved.get("payload"),
        }
        evidence["operations"].append("camera_ptz")

        require_ok(await first.request("control_heartbeat"), "control_heartbeat")
        state = await first.request("get_state", timeout_s=15.0)
        require_ok(state, "get_state")
        for section in ("nav", "route_mission", "patrol_mission", "zones"):
            if section not in state:
                raise RuntimeError(f"state omitted {section}")
        evidence["operations"].append("state")

        zones = await first.request("set_zones_geojson", {
            "geojson": {"type": "FeatureCollection", "features": []}
        }, timeout_s=30.0)
        require_ok(zones, "set_zones_geojson")
        evidence["operations"].append("zones")

        saved = await first.request("save_waypoints_file", {
            "waypoints": [
                {"lat": -31.4858037, "lon": -64.2410570, "role": "home"},
                {"lat": -31.48580, "lon": -64.24100},
            ]
        })
        require_ok(saved, "save_waypoints_file")
        loaded = await first.request("load_waypoints_file")
        require_ok(loaded, "load_waypoints_file")
        if loaded.get("waypoint_count") != 2:
            raise RuntimeError("waypoint round trip changed the count")
        evidence["operations"].append("waypoints")

        require_ok(
            await first.request("set_manual_mode", {"enabled": True}),
            "set_manual_mode",
        )
        require_ok(await first.request("set_manual_cmd", {
            "linear_x": 0.0, "angular_z": 0.0, "brake_pct": 100
        }), "set_manual_cmd")
        require_ok(
            await first.request("set_manual_mode", {"enabled": False}),
            "set_manual_mode",
        )
        evidence["operations"].append("manual_safe_stop")

        snapshot = None
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            # The gateway deliberately gives rendering/service discovery a
            # 20-second budget.  The transport probe must not cancel its recv
            # before that bounded operation can answer.
            snapshot = await first.request("get_nav_snapshot", timeout_s=25.0)
            if snapshot.get("ok") is True:
                break
            await asyncio.sleep(0.5)
        require_ok(snapshot, "get_nav_snapshot")
        if not str(snapshot.get("image_b64", "")).strip():
            raise RuntimeError("snapshot image is empty")
        evidence["operations"].append("snapshot")

        for operation in ("cancel_goal", "cancel_route", "cancel_patrol", "brake"):
            response = await first.request(operation)
            if "ok" not in response:
                raise RuntimeError(f"{operation} returned malformed ack")
        evidence["operations"].append("safe_operations")
        evidence["broadcast_ops"] = sorted({
            str(item.get("op")) for item in first.broadcasts if item.get("op")
        })
        return evidence
    finally:
        if heartbeat_task is not None:
            heartbeat_task.cancel()
            await asyncio.gather(heartbeat_task, return_exceptions=True)
        await second.close()
        await first.close()


def main() -> int:
    report_path = Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "web_cockpit.json"
    success = False
    evidence = {}
    error = None
    try:
        evidence = asyncio.run(scenario())
        success = True
        print("Cockpit WebSocket simulation smoke test passed")
        return 0
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        report_path.write_text(json.dumps({
            "scenario": "web-cockpit",
            "success": success,
            "error": error,
            "evidence": evidence,
        }, indent=2, sort_keys=True), encoding="utf-8")


if __name__ == "__main__":
    sys.exit(main())
