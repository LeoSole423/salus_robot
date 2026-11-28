"""Segment retention and crash recovery. Called only by the recording worker."""
import json
import os
from pathlib import Path
import shutil
import uuid

from .policy import NS


def write_json(path, data):
    temporary = path.with_suffix(".tmp")
    with temporary.open("w") as stream:
        json.dump(data, stream, indent=2)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)


class Archive:
    def __init__(self, root, notify, *, pre=900, post=120, ring_bytes=2 << 30,
                 archive_bytes=10 << 30, free_bytes=10 << 30, context=None, required_topics=()):
        self.root = Path(root)
        self.ring = self.root / "ring"
        self.incidents = self.root / "incidents"
        self.ring.mkdir(parents=True, exist_ok=True)
        self.incidents.mkdir(exist_ok=True)
        self.pre, self.post = pre * NS, post * NS
        self.ring_bytes, self.archive_bytes, self.free_bytes = ring_bytes, archive_bytes, free_bytes
        self.context = context or (lambda: {})
        self.notify = notify
        self.required_topics = set(required_topics)
        self.pending = None
        self.segments = sorted(self.ring.glob("*/segment.json"))
        for path in sorted(self.incidents.glob("*/manifest.json")):
            info = json.loads(path.read_text())
            for temporary in path.parent.glob("*.pending"):
                shutil.rmtree(temporary)
            if info["status"] == "capturing":
                self.pending = (path.parent, info)
                self.preserve()
                self.finish("recorder_restarted")
        for folder in self.ring.iterdir():
            if folder.is_dir() and not (folder / "segment.json").exists():
                shutil.rmtree(folder)

    def latest_stamp(self):
        return json.loads(self.segments[-1].read_text())["end_ns"] if self.segments else None

    def reset_clock(self):
        self.finish("clock_reset")
        for segment in self.segments:
            shutil.rmtree(segment.parent)
        self.segments.clear()

    def trigger(self, stamp, reason, mission, context=None):
        if self.pending and stamp > self.pending[1]["end_ns"]:
            self.finish("overlapping_window")
        if self.pending:
            folder, info = self.pending
        else:
            incident = uuid.uuid4().hex
            folder = self.incidents / incident
            folder.mkdir()
            info = dict(id=incident, status="capturing", mission_id=mission, trigger_ns=stamp,
                        start_ns=stamp - self.pre, end_ns=stamp + self.post, triggers=[],
                        context={**self.context(), **(context or {})}, video_camera="main")
            offset = info["context"].get("wall_time_ns", stamp) - info["context"].get("ros_time_ns", stamp)
            info["video_start_unix_ns"] = info["start_ns"] + offset
            info["video_end_unix_ns"] = info["end_ns"] + offset
            self.pending = (folder, info)
        first = not info["triggers"]
        if all(item["reason"] != reason for item in info["triggers"]):
            info["triggers"].append(dict(stamp_ns=stamp, reason=reason))
            write_json(folder / "manifest.json", info)
        self.preserve()
        if first:
            self.notify(info)

    def add(self, folder, info):
        path = folder / "segment.json"
        write_json(path, info)
        self.segments.append(path)
        self.preserve()
        self.trim_ring(info["end_ns"])

    def trim_ring(self, stamp):
        total = sum(self.segment_size(p) for p in self.segments)
        while self.segments:
            oldest = self.segments[0]
            age = stamp - json.loads(oldest.read_text())["end_ns"]
            if age <= self.pre and total <= self.ring_bytes and self.has_space():
                break
            total -= self.segment_size(oldest)
            shutil.rmtree(self.segments.pop(0).parent)

    @staticmethod
    def segment_size(path):
        return sum(file.stat().st_size for file in path.parent.rglob("*") if file.is_file())

    def preserve(self):
        if not self.pending:
            return
        folder, info = self.pending
        for segment in self.segments:
            target = folder / segment.parent.name
            if target.exists():
                continue
            span = json.loads(segment.read_text())
            if span["end_ns"] >= info["start_ns"] and span["start_ns"] <= info["end_ns"]:
                temporary = target.with_suffix(".pending")
                if temporary.exists():
                    shutil.rmtree(temporary)
                shutil.copytree(segment.parent, temporary, copy_function=os.link)
                temporary.rename(target)

    def advance(self, stamp):
        if self.pending and stamp >= self.pending[1]["end_ns"]:
            self.finish()

    def finish(self, issue=None):
        if not self.pending:
            return
        folder, info = self.pending
        spans = sorted(
            (json.loads(path.read_text()) for path in folder.glob("*/segment.json")),
            key=lambda span: span["start_ns"],
        )
        coverage, dropped, gaps = {}, {}, []
        cursor = info["start_ns"]
        for span in spans:
            if span["start_ns"] > cursor + NS // 4:
                gaps.append([cursor, min(span["start_ns"], info["end_ns"])])
            cursor = max(cursor, span["end_ns"])
            for topic, data in span["topics"].items():
                old = coverage.setdefault(topic, dict(first_ns=data["first_ns"], last_ns=0, count=0))
                old["last_ns"] = max(old["last_ns"], data["last_ns"])
                old["count"] += data["count"]
            for topic, count in span["drops"].items():
                dropped[topic] = dropped.get(topic, 0) + count
        if cursor < info["end_ns"] - NS // 4:
            gaps.append([cursor, info["end_ns"]])
        missing = sorted(self.required_topics - coverage.keys())
        info.update(status="partial" if issue or gaps or dropped or missing else "available",
                    missing_topics=missing,
                    coverage=coverage, drops=dropped, gaps_ns=gaps,
                    actual_start_ns=spans[0]["start_ns"] if spans else None,
                    actual_end_ns=spans[-1]["end_ns"] if spans else None)
        if issue:
            info["issue"] = issue
        write_json(folder / "manifest.json", info)
        self.pending = None
        self.prune()
        if folder.exists():
            self.notify(info)

    def has_space(self):
        return shutil.disk_usage(self.root).free >= self.free_bytes

    def prune(self):
        closed = []
        for path in self.incidents.glob("*/manifest.json"):
            info = json.loads(path.read_text())
            if info["status"] != "capturing":
                closed.append((info["trigger_ns"], path, info, self.segment_size(path)))
        total = sum(item[3] for item in closed)
        for _, path, info, size in sorted(closed):
            if total <= self.archive_bytes and self.has_space():
                break
            shutil.rmtree(path.parent)
            total -= size
            self.notify({**info, "status": "expired"})
