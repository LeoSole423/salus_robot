#!/usr/bin/env python3
"""On-demand video-only RTSP/TCP copy for the frontal camera; no ROS or encoding."""
import argparse
import json
import os
from pathlib import Path
from urllib.parse import urlsplit


PATHS = ("cam3", "cam3_normal")


def build_command(config, path):
    """Build argv without a shell; credentials stay in private configuration."""
    if path not in PATHS:
        raise ValueError("Unsupported frontal path")
    source = config["sources"][path]
    if not isinstance(source, str) or urlsplit(source).scheme != "rtsp" or not urlsplit(source).hostname:
        raise ValueError("Expected an RTSP source")
    binary = config["ffmpeg"]
    if not isinstance(binary, str) or not binary:
        raise ValueError("Expected an FFmpeg executable")
    port = config.get("publish_port", 8554)
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise ValueError("Invalid local RTSP port")
    return [binary, "-nostdin", "-hide_banner", "-loglevel", "quiet",
            "-rtsp_transport", "tcp", "-allowed_media_types", "video",
            "-fflags", "nobuffer", "-analyzeduration", "500000", "-probesize", "524288",
            "-i", source, "-map", "0:v:0", "-c:v", "copy", "-an", "-sn", "-dn",
            "-f", "rtsp", "-rtsp_transport", "tcp",
            f"rtsp://127.0.0.1:{port}/{path}"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--path", choices=PATHS, required=True)
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text())
        argv = build_command(config, args.path)
        env = dict(os.environ)
        if config.get("ffmpeg_library_path"):
            env["LD_LIBRARY_PATH"] = config["ffmpeg_library_path"]
        # Replace the launcher: MediaMTX signals and reaps FFmpeg directly.
        # Quiet stderr avoids disclosing authenticated input URLs in the journal.
        os.execvpe(argv[0], argv, env)
    except (OSError, ValueError, KeyError, TypeError):
        parser.exit(1, "Frontal relay failed; check the private configuration and FFmpeg installation.\n")


if __name__ == "__main__":
    main()
