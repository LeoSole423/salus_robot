"""Manual, non-ROS Hikvision streaming profile tool."""

from __future__ import annotations

import argparse
import json
from typing import Any

from .camera_stream_profile import (
    HikvisionStreamingClient,
    STREAMS_PATH,
    StreamingProfileError,
    config_from_environment,
    load_profile,
    parse_capabilities,
    parse_stream_ids,
    parse_stream_profile,
    validate_capabilities,
    verify_profile,
    apply_profile_xml,
)


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        client = HikvisionStreamingClient(config_from_environment())
        if args.command == "inspect":
            output = _inspect(client, args.stream_id)
        else:
            output = _apply(client, args.stream_id, args.profile, args.dry_run)
        print(json.dumps(output, indent=2, sort_keys=True))
        return 0
    except (StreamingProfileError, RuntimeError) as error:
        parser.exit(2, f"camera stream profile error: {error}\n")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="camera_stream_profile_tool")
    commands = parser.add_subparsers(dest="command", required=True)
    inspect = commands.add_parser("inspect", help="list or inspect camera stream profiles")
    inspect.add_argument("--stream-id", help="inspect one discovered stream")
    apply = commands.add_parser("apply", help="apply a non-secret YAML profile")
    apply.add_argument("--stream-id", required=True)
    apply.add_argument("--profile", required=True, help="non-secret YAML profile")
    apply.add_argument("--dry-run", action="store_true", help="show semantic changes without PUT")
    return parser


def _inspect(client: HikvisionStreamingClient, stream_id: str | None) -> dict[str, Any]:
    ids = parse_stream_ids(client.get(STREAMS_PATH))
    result: dict[str, Any] = {"stream_ids": list(ids)}
    if stream_id is None:
        return result
    if stream_id not in ids:
        raise StreamingProfileError(f"stream id {stream_id} was not discovered")
    xml = client.get(f"{STREAMS_PATH}/{stream_id}")
    capabilities = _get_capabilities(client, stream_id)
    result.update({
        "stream_id": stream_id,
        "profile": parse_stream_profile(xml, stream_id).as_dict(),
        "capabilities": capabilities.as_dict() if capabilities else "unavailable",
    })
    return result


def _apply(
    client: HikvisionStreamingClient,
    stream_id: str,
    profile_path: str,
    dry_run: bool,
) -> dict[str, Any]:
    ids = parse_stream_ids(client.get(STREAMS_PATH))
    if stream_id not in ids:
        raise StreamingProfileError(f"stream id {stream_id} was not discovered")
    desired = load_profile(profile_path)
    path = f"{STREAMS_PATH}/{stream_id}"
    current_xml = client.get(path)
    current = parse_stream_profile(current_xml, stream_id)
    capabilities = _get_capabilities(client, stream_id)
    validate_capabilities(desired, capabilities, current.rate_control)
    changed_xml, changes = apply_profile_xml(current_xml, desired, stream_id)
    result: dict[str, Any] = {
        "stream_id": stream_id,
        "dry_run": dry_run,
        "changes": changes,
        "put": False,
    }
    if dry_run or not changes:
        return result
    client.put(path, changed_xml)
    verified = parse_stream_profile(client.get(path), stream_id)
    verify_profile(verified, desired)
    result["put"] = True
    result["verified"] = verified.as_dict()
    return result


def _get_capabilities(client: HikvisionStreamingClient, stream_id: str):
    try:
        return parse_capabilities(client.get(f"{STREAMS_PATH}/{stream_id}/capabilities"))
    except Exception as error:
        if getattr(error, "status", None) == 404:
            return None
        raise


if __name__ == "__main__":
    raise SystemExit(main())
