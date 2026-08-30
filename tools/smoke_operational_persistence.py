#!/usr/bin/env python3
"""Seed and verify Cockpit-owned state across a minimal Web + Camera restart."""

import argparse
import asyncio
import json
import os
from pathlib import Path

from smoke_web_cockpit import CockpitProbe, require_ok


WAYPOINTS = [
    {"lat": -31.4858037, "lon": -64.2410570, "role": "home"},
    {"lat": -31.48580, "lon": -64.24100},
]


async def seed() -> dict:
    port = int(os.environ["SALUS_WEB_SMOKE_PORT"])
    client = CockpitProbe(f"ws://127.0.0.1:{port}")
    try:
        initial = await client.connect()
        if initial.get("op") != "state":
            raise RuntimeError(f"invalid initial state: {initial}")
        unlocked = await client.request("set_control_lock", {"locked": False})
        require_ok(unlocked, "unlock persistence seed")

        saved = await client.request("save_waypoints_file", {"waypoints": WAYPOINTS})
        require_ok(saved, "save_waypoints_file")
        if saved.get("waypoint_count") != len(WAYPOINTS):
            raise RuntimeError(f"waypoint seed count mismatch: {saved}")

        moved = await client.request(
            "camera_ptz_move",
            {
                "relative": False,
                "pan_deg": 25.0,
                "tilt_deg": 10.0,
                "zoom_level": 2.0,
            },
        )
        require_ok(moved, "camera_ptz_move seed")
        require_ok(await client.request("control_heartbeat"), "control_heartbeat")
        preset = await client.request(
            "camera_ptz_set_preset", {"preset": "home", "save_zoom": True}
        )
        require_ok(preset, "camera_ptz_set_preset seed")
        return {
            "waypoint_count": saved["waypoint_count"],
            "seeded_camera": moved.get("payload", {}),
        }
    finally:
        await client.close()


async def verify() -> dict:
    port = int(os.environ["SALUS_WEB_SMOKE_PORT"])
    client = CockpitProbe(f"ws://127.0.0.1:{port}")
    try:
        initial = await client.connect()
        if initial.get("op") != "state":
            raise RuntimeError(f"invalid restored initial state: {initial}")
        waypoints = await client.request("load_waypoints_file")
        require_ok(waypoints, "load_waypoints_file after restart")
        if waypoints.get("waypoint_count") != len(WAYPOINTS):
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("seed", "verify"), required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report_path = (
        Path(os.environ.get("SMOKE_ARTIFACT_DIR", "."))
        / f"persistence_{args.mode}.json"
    )
    success = False
    evidence = {}
    error = None
    try:
        evidence = asyncio.run(seed() if args.mode == "seed" else verify())
        success = True
        print(f"Operational persistence {args.mode} passed")
        return 0
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        report_path.write_text(
            json.dumps(
                {
                    "scenario": "operational-persistence",
                    "mode": args.mode,
                    "success": success,
                    "error": error,
                    "evidence": evidence,
                },
                indent=2,
                sort_keys=True,
            ),
            encoding="utf-8",
        )


if __name__ == "__main__":
    raise SystemExit(main())
