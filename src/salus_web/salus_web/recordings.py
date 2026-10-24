"""MediaMTX boundary. Control stays behind the Cockpit operator lease.

Media servers listen on loopback; signed, short-lived downloads are served by
this read-only proxy on the configured operator-network address.
"""
import hashlib
import hmac
import json
import math
import os
import re
from pathlib import Path
import secrets
import shutil
import subprocess
import threading
import time
from datetime import datetime, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlencode, urlsplit, parse_qs
from urllib.request import Request, urlopen

from .recording_policy import RecordingPolicy

CAMERAS = {"main": "rec_cam3", "mobile": "rec_cam4", "rear": "rec_cam5"}


def timestamp(value):
    """Parse RFC3339Nano from MediaMTX on Python 3.10 (microsecond precision)."""
    if not isinstance(value, str):
        raise ValueError("timezone_required")
    value = re.sub(r"\.(\d+)(?=Z|[+-]\d{2}:\d{2}$|$)",
                   lambda match: "." + match[1][:6].ljust(6, "0"), value)
    result = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if result.tzinfo is None:
        raise ValueError("timezone_required")
    return result


def http_json(url, data=None, method=None):
    body = None if data is None else json.dumps(data).encode()
    req = Request(url, data=body, method=method,
                  headers={"Content-Type": "application/json"})
    with urlopen(req, timeout=8) as response:
        raw = response.read(4 * 1024 * 1024)
    return json.loads(raw) if raw else {}


class Recordings:
    def __init__(self, config):
        self.config = config
        self.lock = threading.RLock()
        self.key = secrets.token_bytes(32)
        self.state_file = Path(config["state_file"])
        self.root = Path(config["root"]).resolve()
        self.policy = RecordingPolicy()
        if self.state_file.exists():
            saved = json.loads(self.state_file.read_text())
            self.policy.manual = saved.get("manual") is True
            self.policy.mission = saved.get("mission") is True
            if saved.get("tail_pending") is True:
                self.policy.tail_until = time.monotonic() + 30
        self.processes = {}
        self.errors = {}
        self.applied = {}
        self.ready = {}
        self.disk_ok = True
        self.stop_event = threading.Event()
        self.server = None

    def persist(self):
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.state_file.with_suffix(".tmp")
        with tmp.open("w") as stream:
            json.dump({"manual": self.policy.manual, "mission": self.policy.mission,
                       "tail_pending": self.policy.tail_until > time.monotonic()}, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, self.state_file)

    def observe(self, active):
        with self.lock:
            self.policy.observe(active, time.monotonic())
            self.persist()

    def manual(self, enabled):
        with self.lock:
            previous = self.policy.manual
            self.policy.manual = enabled
            try:
                self.persist()
            except Exception:
                self.policy.manual = previous
                raise
        return self.status()

    def status(self):
        with self.lock:
            now = time.monotonic()
            return {"manual": self.policy.manual, "mission": self.policy.mission,
                    "tail_seconds": max(0, math.ceil(self.policy.tail_until - now)),
                    "mission_stale": self.policy.stale,
                    "requested": self.policy.wanted(now), "disk_ok": self.disk_ok,
                    "cameras": [{"camera": camera, "recording": bool(
                        self.applied.get(path) and self.ready.get(path) and
                        any(time.time() - p.stat().st_mtime < 10 and p.stat().st_size > 0
                            for p in (self.root / path).glob("*.mp4") if not p.is_symlink())),
                        "error": self.errors.get(path, "")} for camera, path in CAMERAS.items()]}

    def catalog(self, fields):
        camera = fields.get("camera", "all")
        if camera not in {"all", *CAMERAS}:
            raise ValueError("invalid_camera")
        start, end = fields.get("start"), fields.get("end")
        begin, finish = timestamp(start), timestamp(end)
        if not 0 < (finish - begin).total_seconds() <= 90000:
            raise ValueError("query_one_day_at_a_time")
        rows = []
        for name, path in CAMERAS.items():
            if camera not in {"all", name}:
                continue
            periods = http_json(self.config["playback"] + "/list?" + urlencode(
                {"path": path, "start": start, "end": end}))
            for period in periods:
                stamp = timestamp(period["start"])
                duration = float(period["duration"])
                if not math.isfinite(duration) or duration <= 0:
                    continue
                # Clamp periods returned by MediaMTX to the selected local day.
                offset = max(0, (begin - stamp).total_seconds())
                limit = min(duration, (finish - stamp).total_seconds())
                while offset < limit:
                    length = min(600, limit - offset)
                    query = urlencode({"path": path, "start": (stamp + timedelta(seconds=offset)).isoformat(),
                                       "duration": length, "format": "mp4", "expires": int(time.time()) + 600})
                    sig = hmac.new(self.key, query.encode(), hashlib.sha256).hexdigest()
                    rows.append({"camera": name, "start": (stamp + timedelta(seconds=offset)).isoformat(),
                                 "duration": length, "url": self.config["public_url"] + "/download?" + query + "&sig=" + sig})
                    offset += length
                    if len(rows) > 1000:
                        raise ValueError("catalog_too_large")
        return {"items": sorted(rows, key=lambda row: row["start"], reverse=True), "status": self.status()}

    def maintain_disk(self):
        # Only delete files MediaMTX itself reports as completed/closed:
        # conservatively exclude the newest segment of EACH stream, and any
        # file modified in the last 120 s. This also protects unavailable streams.
        self.root.mkdir(parents=True, exist_ok=True)
        files = []
        now = time.time()
        for path in CAMERAS.values():
            directory = self.root / path
            if directory.is_symlink():
                continue
            candidates = sorted((p for p in directory.glob("*.mp4") if not p.is_symlink()),
                                key=lambda p: p.stat().st_mtime)
            files.extend(p for p in candidates[:-1] if now - p.stat().st_mtime > 120)
        for file in sorted(files, key=lambda p: p.stat().st_mtime):
            if shutil.disk_usage(self.root).free >= 20 * 1024**3:
                break
            file.unlink()
        self.disk_ok = shutil.disk_usage(self.root).free >= 20 * 1024**3

    def tick(self):
        try:
            self.maintain_disk()
        except OSError:
            self.disk_ok = False
        with self.lock:
            wanted = self.policy.wanted(time.monotonic()) and self.disk_ok
        for path in CAMERAS.values():
            try:
                process = self.processes.get(path)
                if not wanted and process:
                    process.terminate()
                    try:
                        process.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3)
                    self.processes.pop(path, None)
                if wanted and (process is None or process.poll() is not None):
                    source = self.config.get("rtsp", "rtsp://127.0.0.1:8554") + "/" + path.removeprefix("rec_")
                    target = self.config.get("rtsp", "rtsp://127.0.0.1:8554") + "/" + path
                    self.processes[path] = subprocess.Popen([
                        "setpriv", "--pdeathsig", "TERM",
                        self.config.get("ffmpeg", "ffmpeg"), "-nostdin", "-loglevel", "error",
                        "-rtsp_transport", "tcp", "-i", source, "-map", "0:v:0", "-an",
                        "-c:v", "copy", "-f", "rtsp", "-rtsp_transport", "tcp", target,
                    ], stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    env={**os.environ, **({"LD_LIBRARY_PATH": self.config["ffmpeg_library_path"]}
                                         if self.config.get("ffmpeg_library_path") else {})})
                info = http_json(self.config["api"] + "/v3/config/paths/get/" + path)
                if info.get("record") != wanted:
                    http_json(self.config["api"] + "/v3/config/paths/patch/" + path,
                              {"record": wanted}, "PATCH")
                info = http_json(self.config["api"] + "/v3/paths/get/" + path)
                with self.lock:
                    self.applied[path] = wanted
                    self.ready[path] = bool(info.get("ready"))
                    self.errors[path] = "" if self.ready[path] or not wanted else "camera_unavailable"
            except Exception:
                with self.lock:
                    self.applied.pop(path, None)
                    self.ready[path] = False
                    self.errors[path] = "media_service_unavailable"

    def start(self):
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass  # Signed URL is a short-lived credential; never log it.

            def do_GET(self):
                parts = urlsplit(self.path)
                raw, separator, sig = parts.query.rpartition("&sig=")
                try:
                    if not separator or parts.path != "/download":
                        self.send_response(403)
                        self.end_headers()
                        return
                    fields = parse_qs(raw, strict_parsing=True)
                    valid = hmac.compare_digest(sig, hmac.new(owner.key, raw.encode(), hashlib.sha256).hexdigest())
                    if parts.path != "/download" or not separator or not valid or int(fields["expires"][0]) < time.time():
                        self.send_response(403)
                        self.end_headers()
                        return
                    query = {name: fields[name][0] for name in ("path", "start", "duration", "format")}
                    if query["path"] not in CAMERAS.values() or not 0 < float(query["duration"]) <= 600:
                        self.send_response(400)
                        self.end_headers()
                        return
                    with urlopen(owner.config["playback"] + "/get?" + urlencode(query), timeout=30) as response:
                        self.send_response(200)
                        self.send_header("Content-Type", "video/mp4")
                        filename = query["path"] + "_" + str(int(time.time())) + ".mp4"
                        self.send_header("Content-Disposition", 'attachment; filename="' + filename + '"')
                        self.send_header("Cache-Control", "no-store")
                        self.send_header("Referrer-Policy", "no-referrer")
                        self.end_headers()
                        shutil.copyfileobj(response, self.wfile, 65536)
                except Exception:
                    if not self.wfile.closed:
                        # A midstream interruption must remain a failed download.
                        self.close_connection = True

        self.server = ThreadingHTTPServer((self.config["bind"], int(self.config["port"])), Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

        def loop():
            while not self.stop_event.is_set():
                try:
                    self.tick()
                except Exception:
                    with self.lock:
                        self.disk_ok = False
                        self.errors = {path: "recording_storage_error" for path in CAMERAS.values()}
                self.stop_event.wait(2)
        self.worker = threading.Thread(target=loop, daemon=True)
        self.worker.start()

    def close(self):
        self.stop_event.set()
        if getattr(self, "worker", None):
            self.worker.join(timeout=30)
        for process in self.processes.values():
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        if self.server:
            self.server.shutdown()
            self.server.server_close()
