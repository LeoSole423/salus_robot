import json
import time
from pathlib import Path

import pytest
from salus_web import recordings
from salus_web.protocol import is_controlled_operation, parse_request, validate_request


def manager(tmp_path):
    return recordings.Recordings({"state_file": str(tmp_path / "state.json"),
        "root": str(tmp_path / "videos"), "api": "http://127.0.0.1:9997",
        "playback": "http://127.0.0.1:9996", "public_url": "http://127.0.0.1:8790"})


def test_control_requires_operator_lease():
    for op in ("recordings_start", "recordings_stop"):
        assert is_controlled_operation(validate_request(parse_request({"op": op})))
    assert not is_controlled_operation(validate_request(parse_request({"op": "recordings_list"})))


def test_restart_preserves_manual_and_unknown_mission(tmp_path):
    m = manager(tmp_path)
    m.manual(True)
    m.observe(True)
    restored = manager(tmp_path)
    assert restored.status()["requested"]
    restored.manual(False)
    assert restored.status()["requested"]
    restored.observe(False)
    assert restored.status()["tail_seconds"] == 30


def test_catalog_limits_duration_and_signs_downloads(tmp_path, monkeypatch):
    monkeypatch.setattr(recordings, "http_json", lambda *args: [{"start": "2026-10-24T00:00:00Z", "duration": 1300}])
    result = manager(tmp_path).catalog({"camera": "main", "start": "2026-10-24T00:00:00Z", "end": "2026-10-25T00:00:00Z"})
    assert [r["duration"] for r in result["items"]] == [100, 600, 600]
    assert all("&sig=" in r["url"] for r in result["items"])


def test_bad_catalog_input(tmp_path):
    with pytest.raises(ValueError):
        manager(tmp_path).catalog({"camera": "../secrets"})
    with pytest.raises(ValueError):
        manager(tmp_path).catalog({"start": "2026-10-24", "end": "2026-10-25"})


def test_low_disk_never_deletes_newest_or_symlink(tmp_path, monkeypatch):
    m = manager(tmp_path)
    folder = m.root / "rec_cam3"
    folder.mkdir(parents=True)
    older, newest = folder / "a.mp4", folder / "b.mp4"
    older.write_bytes(b"old")
    newest.write_bytes(b"new")
    import os
    os.utime(older, (1, 1))
    os.utime(newest, (2, 2))
    (folder / "c.mp4").symlink_to(older)
    monkeypatch.setattr(recordings.shutil, "disk_usage", lambda _: type("Usage", (), {"free": 0})())
    m.maintain_disk()
    assert not older.exists()
    assert newest.exists()
    assert (folder / "c.mp4").is_symlink()
    assert not m.disk_ok


def test_camera_failure_not_reported_as_recording(tmp_path, monkeypatch):
    m = manager(tmp_path)
    m.manual(True)
    def unavailable(*args):
        raise OSError("offline")
    monkeypatch.setattr(recordings, "http_json", unavailable)
    m.tick()
    assert all(not c["recording"] and c["error"] for c in m.status()["cameras"])


def test_download_rejects_unsigned_requests(tmp_path):
    import urllib.error
    import urllib.request
    m = manager(tmp_path)
    m.config.update(bind="127.0.0.1", port=0)
    m.start()
    try:
        url = 'http://127.0.0.1:%s/download?sig=invalid' % m.server.server_port
        with pytest.raises(urllib.error.HTTPError) as result:
            urllib.request.urlopen(url, timeout=2)
        assert result.value.code == 403
    finally:
        m.close()


@pytest.mark.parametrize("fraction", ["1", "12", "12345", "123456789"])
def test_mediamtx_rfc3339nano_fraction_precision(fraction):
    assert recordings.timestamp('2026-10-24T17:18:18.' + fraction + '-03:00').microsecond == int(fraction[:6].ljust(6, '0'))


def test_space_cleanup_does_not_follow_camera_directory_symlinks(tmp_path, monkeypatch):
    m = manager(tmp_path)
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'a.mp4').write_bytes(b'preserve')
    (outside / 'b.mp4').write_bytes(b'preserve')
    m.root.mkdir()
    (m.root / 'rec_cam3').symlink_to(outside, target_is_directory=True)
    monkeypatch.setattr(recordings.shutil, 'disk_usage', lambda _: type('Usage', (), {'free': 0})())
    m.maintain_disk()
    assert (outside / 'a.mp4').exists()
