#!/usr/bin/env python3
"""Record a reproducible identity report for an externally stored LiDAR bag."""
from __future__ import annotations
import argparse, hashlib, json
from pathlib import Path
parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("bag",type=Path);parser.add_argument("--output",type=Path,default=Path("lidar-replay-report.json"));args=parser.parse_args()
if not args.bag.exists(): parser.error("bag path does not exist")
digest=hashlib.sha256()
for file in sorted(path for path in args.bag.rglob("*") if path.is_file()):
    digest.update(file.relative_to(args.bag).as_posix().encode());digest.update(file.read_bytes())
args.output.write_text(json.dumps({"bag":str(args.bag),"sha256":digest.hexdigest(),"status":"input recorded; run lidar_diagnostics.launch.py during replay"},indent=2)+"\n")
