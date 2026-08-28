from salus_hardware.rtk_domain import crc24q, evaluate_rtcm_delivery


def frame(payload=b"\x01\x02"):
    header = (0xD3, (len(payload) >> 8) & 0x03, len(payload) & 0xFF)
    prefix = bytes(header) + payload
    return prefix + crc24q(prefix).to_bytes(3, "big")


def test_delivery_accepts_only_advancing_sequences_per_source():
    first = evaluate_rtcm_delivery(
        data=frame(), source_id="base", sequence=7,
        previous_source_id=None, previous_sequence=None,
    )
    assert first.accepted
    duplicate = evaluate_rtcm_delivery(
        data=frame(), source_id="base", sequence=7,
        previous_source_id=first.source_id, previous_sequence=first.sequence,
    )
    assert not duplicate.accepted
    assert duplicate.reason == "sequence_duplicate"
    reset = evaluate_rtcm_delivery(
        data=frame(), source_id="base", sequence=1,
        previous_source_id=first.source_id, previous_sequence=first.sequence,
    )
    assert not reset.accepted
    assert reset.reason == "sequence_reset"
    assert reset.sequence == 1
    after_reset = evaluate_rtcm_delivery(
        data=frame(), source_id="base", sequence=2,
        previous_source_id=reset.source_id, previous_sequence=reset.sequence,
    )
    assert after_reset.accepted


def test_source_change_establishes_a_new_sequence_domain():
    decision = evaluate_rtcm_delivery(
        data=frame(), source_id="other_base", sequence=1,
        previous_source_id="base", previous_sequence=99,
    )
    assert decision.accepted
    assert decision.source_id == "other_base"
    assert decision.sequence == 1


def test_invalid_or_anonymous_frames_never_advance_delivery_state():
    invalid = evaluate_rtcm_delivery(
        data=b"", source_id="base", sequence=2,
        previous_source_id="base", previous_sequence=1,
    )
    assert not invalid.accepted
    assert invalid.sequence == 1
    anonymous = evaluate_rtcm_delivery(
        data=frame(), source_id="", sequence=2,
        previous_source_id="base", previous_sequence=1,
    )
    assert not anonymous.accepted
    assert anonymous.reason == "missing_source_id"


def test_valid_rtcm_larger_than_mavros_limit_is_not_delivered():
    decision = evaluate_rtcm_delivery(
        data=frame(bytes(715)), source_id="base", sequence=2,
        previous_source_id="base", previous_sequence=1,
    )
    assert not decision.accepted
    assert decision.reason == "mavros_payload_too_large"
