from salus_web.recording_policy import RecordingPolicy


def test_manual_and_mission_are_independent():
    p = RecordingPolicy(manual=True)
    p.observe(True, 0)
    p.observe(False, 10)
    assert p.wanted(100)
    p.manual = False
    assert not p.wanted(100)


def test_tail_and_consecutive_mission():
    p = RecordingPolicy()
    p.observe(True, 0)
    p.observe(False, 10)
    assert p.wanted(39)
    assert not p.wanted(40)
    p.observe(True, 39)
    assert p.wanted(100)
    p.manual = False
    assert p.wanted(100)


def test_unknown_does_not_end_mission_or_tail():
    p = RecordingPolicy()
    assert not p.wanted(0)
    p.observe(True, 0)
    p.observe(None, 10)
    assert p.wanted(100)
    p.observe(False, 110)
    p.observe(None, 120)
    assert p.wanted(200)
    p.observe(False, 201)
    assert not p.wanted(201)


def test_paused_and_home_legs_remain_recording():
    from salus_web.recording_policy import mission_signal
    assert mission_signal([{"ok": True, "paused": True}, None]) is True
    assert mission_signal([{"ok": True, "return_home_active": True}, {"ok": True}]) is True
    assert mission_signal([{"ok": True}, None]) is None
    assert mission_signal([{"ok": True}, {"ok": True}]) is False
