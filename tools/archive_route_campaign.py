#!/usr/bin/env python3
"""Archive reproducible route bundles; keep runtime caches out of Git evidence."""
import argparse
import gzip
import hashlib
import io
import json
from pathlib import Path
import tarfile


def archive(source, output):
    output.mkdir(parents=True, exist_ok=True)
    hashes = {}
    payload = io.BytesIO()
    with tarfile.open(fileobj=payload, mode='w') as tar:
        for path in sorted(source.rglob('*')):
            if not path.is_file() or 'runtime' in path.relative_to(source).parts:
                continue
            raw = path.read_bytes()
            relative = str(path.relative_to(source))
            hashes[relative] = hashlib.sha256(raw).hexdigest()
            info = tarfile.TarInfo(relative)
            info.size = len(raw); info.mode = 0o644; info.mtime = 0
            tar.addfile(info, io.BytesIO(raw))
    encoded = gzip.compress(payload.getvalue(), compresslevel=6, mtime=0)
    (output/'bundle.tar.gz').write_bytes(encoded)
    (output/'files-sha256.json').write_text(json.dumps(hashes, indent=2)+'\n')
    (output/'archive-sha256.json').write_text(json.dumps(dict(
        sha256=hashlib.sha256(encoded).hexdigest(), files=len(hashes),
        excluded='runtime process/log cache; evaluation bundles and launch logs retained'),
        indent=2)+'\n')
    return hashes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    archive(args.source, args.output)


if __name__ == '__main__':
    main()
