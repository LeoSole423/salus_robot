#!/usr/bin/env python3
"""Prepare a private MediaMTX config without changing encoders or active services."""
import argparse
import copy
import os
from pathlib import Path
from urllib.parse import urlsplit, urlunsplit

import yaml


def add_quality_paths(config):
    """Retain all existing routes, adding on-demand principal-stream aliases."""
    result = copy.deepcopy(config)
    paths = result.get("paths", {})
    for name in ("cam3", "cam4", "cam5"):
        original = paths.get(name, {})
        source = original.get("source", "")
        parsed = urlsplit(source)
        if parsed.scheme != "rtsp" or not parsed.path.endswith("/video2"):
            raise ValueError(f"{name}: expected a secondary RTSP /video2 source")
        alias = name + "_normal"
        if alias in paths:
            raise ValueError(f"{alias}: already exists; review manually")
        # Copy source transport/auth only, never recording or publisher hooks.
        route = {key: original[key] for key in ("rtspTransport", "rtspAnyPort", "sourceFingerprint") if key in original}
        route.update(source=urlunsplit(parsed._replace(path=parsed.path[:-1] + "1")),
                     sourceOnDemand=True, sourceOnDemandCloseAfter="10s")
        paths[alias] = route
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    try:
        result = add_quality_paths(yaml.safe_load(args.input.read_text()))
        # The resulting config contains the original private credentials.
        fd = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, "w") as output:
            yaml.safe_dump(result, output, sort_keys=False)
    except (ValueError, OSError, yaml.YAMLError):
        parser.exit(1, "Preparation failed; check paths, input YAML and output permissions privately.\n")
    print("Prepared three normal-quality paths; no service or camera was changed.")


if __name__ == "__main__":
    main()
