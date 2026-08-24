#!/usr/bin/env python3
"""Verify Cockpit-owned operational state survives a bringup restart."""

import asyncio
import json
import os
from pathlib import Path

from smoke_web_cockpit import CockpitProbe, require_ok


async def scenario() -> dict:
    port = int(os.environ["SALUS_WEB_SMOKE_PORT"])
    client = CockpitProbe(f"ws://127.0.0.1:{port}")
    try:
        initial = await client.connect()
        if initial.get("op") != "state":
            raise RuntimeError(f"invalid restored initial state: {initial}")
        waypoints = await client.request("load_waypoints_file")
        require_ok(waypoints, "load_waypoints_file after restart")
        if waypoints.get("waypoint_count") != 2:
            raise RuntimeError(f"waypoints did not survive restart: {waypoints}")
        unlocked = await client.request("set_control_lock", {"locked": False})
        require_ok(unlocked, "unlock restored control lease")
        restored = await client.request("camera_ptz_preset", {"preset": "home"})
        require_ok(restored, "camera_ptz_preset after restart")
        state = restored.get("payload", {})
        if abs(float(state.get("pan_deg", -1.0)) - 25.0) > 0.01:
            raise RuntimeError(f"camera preset did not survive restart: {restored}")
        return {
            "waypoint_count": waypoints["waypoint_count"],
            "restored_camera": state,
        }
    finally:
        await client.close()


def main() -> int:
    report_path = Path(os.environ.get("SMOKE_ARTIFACT_DIR", ".")) / "persistence_probe.json"
    success = False
    evidence = {}
    error = None
    try:
        evidence = asyncio.run(scenario())
        success = True
        print("Operational persistence smoke test passed")
        return 0
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        report_path.write_text(json.dumps({
            "scenario": "operational-persistence",
            "success": success,
            "error": error,
            "evidence": evidence,
        }, indent=2, sort_keys=True), encoding="utf-8")


if __name__ == "__main__":
    raise SystemExit(main())
