import json
import shutil

from salus_incident.archive import Archive
from salus_incident.policy import NS, SteeringJump, navigation_reason


def segment(archive, start, end, drops=None):
    folder = archive.ring / f"{start:04}"
    folder.mkdir()
    (folder / "data.db3").write_bytes(b"example")
    archive.add(folder, dict(start_ns=start * NS, end_ns=end * NS, topics={}, drops=drops or {}))
    return folder


def current(archive):
    path, _ = archive.pending
    return path


def manifest(path):
    return json.loads((path / "manifest.json").read_text())


def test_steering_jump_from_incident():
    jump = SteeringJump()
    stamp = 1795825843447082930
    assert not jump.observe(stamp, 1, 1.6, -.028)
    assert jump.observe(stamp + 31_870_494, 1, 1.6, -.361)
    assert not jump.observe(stamp + 50_000_000, 2, 1.6, -.8)
    assert not jump.observe(stamp + 70_000_000, 1, .4, -.8)
    assert navigation_reason("GOAL_RESULT_ABORTED", {"failure_code": "NO_VALID_PATH"}) == "NO_VALID_PATH"
    assert navigation_reason("ROUTE_BLOCKED_NEEDS_OPERATOR", {"reason": "PLANNER_U_TURN"}) == "PLANNER_U_TURN"


def test_rotation_deduplication_and_release(tmp_path):
    events = []
    archive = Archive(tmp_path, events.append, pre=60, post=30, free_bytes=0)
    original = segment(archive, 30, 60)
    segment(archive, 60, 90)
    archive.trigger(90 * NS, "STEERING", "mission")
    output = current(archive)
    archive.trigger(91 * NS, "STEERING", "mission")
    archive.trigger(92 * NS, "COLLISION", "mission")
    assert (original / "data.db3").stat().st_ino == (output / "0030/data.db3").stat().st_ino
    segment(archive, 90, 120)
    archive.advance(120 * NS)
    segment(archive, 120, 150)
    assert not original.exists()
    assert (output / "0030/data.db3").exists()
    info = manifest(output)
    assert info["status"] == "available"
    assert [item["reason"] for item in info["triggers"]] == ["STEERING", "COLLISION"]


def test_restart_recovers_partial_with_drops(tmp_path):
    archive = Archive(tmp_path, lambda _: None, pre=60, post=30, free_bytes=0)
    segment(archive, 60, 90, {"/scan": 2})
    archive.trigger(90 * NS, "COLLISION", "mission")
    output = current(archive)
    recovered = []
    Archive(tmp_path, recovered.append, pre=60, post=30, free_bytes=0)
    info = manifest(output)
    assert info["status"] == "partial"
    assert info["issue"] == "recorder_restarted"
    assert info["drops"] == {"/scan": 2}
    assert info["gaps_ns"] == [[30 * NS, 60 * NS], [90 * NS, 120 * NS]]


def test_disk_floor_prunes_closed_incident(tmp_path, monkeypatch):
    events = []
    archive = Archive(tmp_path, events.append, pre=5, post=5, free_bytes=1)
    segment(archive, 5, 10)
    archive.trigger(10 * NS, "COLLISION", "")
    output = current(archive)
    segment(archive, 10, 15)
    archive.advance(15 * NS)
    monkeypatch.setattr(shutil, "disk_usage", lambda _: shutil._ntuple_diskusage(100, 100, 0))
    assert not archive.has_space()
    archive.prune()
    assert not output.exists()
    assert events[-1]["status"] == "expired"


def test_disk_pressure_releases_ring_without_closed_incidents(tmp_path, monkeypatch):
    archive = Archive(tmp_path, lambda _: None, pre=60, post=30, free_bytes=1)
    oldest = segment(archive, 30, 60)
    monkeypatch.setattr(shutil, "disk_usage", lambda _: shutil._ntuple_diskusage(100, 100, 0))
    archive.trim_ring(60 * NS)
    assert not oldest.exists()
