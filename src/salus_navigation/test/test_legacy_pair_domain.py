from math import nan

import pytest

from salus_navigation.route_checkpoint_tracker import (
    CheckpointOccurrence,
    IntermediateCheckpointTracker,
    PoseSample,
)
from salus_navigation.route_chunker import build_chunk, next_start
from salus_navigation.route_model import RouteWaypoint
from salus_navigation.route_preparation import prepare


def point(x, index, *, role="normal", action="", explicit=False, key=True):
    return RouteWaypoint(
        0.0, 0.0, float(index * 10) if explicit else nan, index,
        key=key, action_json=action, role=role,
        map_x=float(x), map_y=0.0, yaw_explicit=explicit,
    )


def route(points, *, loop=False):
    return prepare(points, loop=loop, input_count=len(points), spacing_m=0.0,
                   chunk_span_m=120.0, chunk_max_waypoints=5)


def indices(chunk):
    return [item.input_index for item in chunk.waypoints if item.key]


def test_legacy_pair_builds_disjoint_open_pairs_and_final_singleton():
    prepared = route([point(0, 0), point(10, 1), point(20, 2), point(30, 3)])
    first = build_chunk(prepared, 0, mode="legacy_pair")
    second = build_chunk(prepared, next_start(prepared, first), mode="legacy_pair")

    assert indices(first) == [0, 1]
    assert indices(second) == [2, 3]
    assert next_start(prepared, second) == 4


def test_legacy_pair_preserves_synthetic_horizon_until_next_real_key():
    prepared = route([point(0, 0), point(10, 1), point(20, 2)])
    prepared = prepare(list(prepared.waypoints), loop=False, input_count=3,
                       spacing_m=4.0, chunk_span_m=1.0, chunk_max_waypoints=1)
    chunk = build_chunk(prepared, 0, mode="legacy_pair")

    assert indices(chunk) == [0, 1]
    assert chunk.waypoints[-1].input_index == 1
    assert chunk.waypoints[-1].key


def test_legacy_pair_hard_start_is_singleton_and_explicit_intermediate_is_hard():
    prepared = route([point(0, 0, action='{"type":"brake_hold"}'), point(10, 1), point(20, 2)])
    chunk = build_chunk(prepared, 0, mode="legacy_pair")
    assert indices(chunk) == [0]

    prepared = route([point(0, 0, explicit=True), point(10, 1), point(20, 2)])
    chunk = build_chunk(prepared, 0, mode="legacy_pair")
    assert indices(chunk) == [0]

    prepared = route([point(0, 0), point(10, 1, explicit=True), point(20, 2)])
    first = build_chunk(prepared, 0, mode="legacy_pair")
    assert indices(first) == [0, 1]


def test_legacy_pair_loop_closure_tracks_occurrence_iterations():
    prepared = route([point(0, 0), point(10, 1), point(20, 2)], loop=True)
    first = build_chunk(prepared, 2, iteration=4, mode="legacy_pair")

    assert indices(first) == [2, 0]
    assert first.checkpoint_occurrences == ((0, 2, 4), (1, 0, 5))


def test_single_checkpoint_mode_remains_one_real_key():
    prepared = route([point(0, 0), point(10, 1), point(20, 2)])
    chunk = build_chunk(prepared, 0)
    assert indices(chunk) == [0]


def sample(x, stamp=10.0, received=10.0):
    return PoseSample(x, 0.0, stamp, received)


def test_tracker_accepts_one_fresh_soft_checkpoint_in_order_and_once():
    tracker = IntermediateCheckpointTracker((CheckpointOccurrence(1, 0, 10.0, 0.0),))
    accepted = tracker.observe(sample(8.0), now_ros_s=10.1, now_steady_s=10.1)
    repeated = tracker.observe(sample(10.0, stamp=10.1, received=10.1), now_ros_s=10.2, now_steady_s=10.2)
    assert accepted.accepted and accepted.distance_m == pytest.approx(2.0)
    assert not repeated.accepted and repeated.reason == "no_pending_checkpoint"
    assert tracker.minimum_distances_m == (pytest.approx(2.0),)


def test_tracker_rejects_stale_future_invalid_and_non_monotonic_samples():
    tracker = IntermediateCheckpointTracker((CheckpointOccurrence(1, 0, 10.0, 0.0),))
    assert tracker.observe(sample(10.0, stamp=9.0, received=10.0), now_ros_s=10.0, now_steady_s=10.0).reason == "stale_pose"
    assert tracker.observe(sample(10.0, stamp=10.2, received=10.1), now_ros_s=10.0, now_steady_s=10.1).reason == "future_or_clock_regression"
    assert tracker.observe(sample(10.0, stamp=10.0, received=10.0), now_ros_s=10.1, now_steady_s=10.1).accepted
    assert tracker.observe(sample(10.0, stamp=10.0, received=10.1), now_ros_s=10.2, now_steady_s=10.2).reason == "non_monotonic_source_stamp"


def test_tracker_does_not_acknowledge_out_of_radius_or_invalid_pose():
    tracker = IntermediateCheckpointTracker((CheckpointOccurrence(1, 0, 10.0, 0.0),))
    far = tracker.observe(sample(0.0), now_ros_s=10.1, now_steady_s=10.1)
    invalid = tracker.observe(PoseSample(float("nan"), 0.0, 11.0, 11.0), now_ros_s=11.1, now_steady_s=11.1)
    assert not far.accepted and far.reason == "outside_radius"
    assert not invalid.accepted and invalid.reason == "invalid_pose_sample"


def test_tracker_requires_each_ordered_occurrence_before_terminal_is_ready():
    tracker = IntermediateCheckpointTracker((
        CheckpointOccurrence(1, 0, 10.0, 0.0),
        CheckpointOccurrence(2, 0, 20.0, 0.0),
    ))
    second_first = tracker.observe(sample(20.0), now_ros_s=10.1, now_steady_s=10.1)
    assert not second_first.accepted and second_first.reason == "outside_radius"
    assert not tracker.complete
    first = tracker.observe(sample(10.0, stamp=10.1, received=10.1), now_ros_s=10.2, now_steady_s=10.2)
    second = tracker.observe(sample(20.0, stamp=10.2, received=10.2), now_ros_s=10.3, now_steady_s=10.3)
    assert first.accepted and second.accepted and tracker.complete
