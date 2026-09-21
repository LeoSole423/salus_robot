"""Pure tests for the obstacle-drag static scan geometry metrics."""

import math
from types import SimpleNamespace

import pytest

from salus_evaluation.models import Pose2D
from salus_evaluation.stage_metrics import (
    beam_support_is_identical, exact_common_stamps, pointcloud_payload_signature,
    compare_scan_projection, project_pointcloud_to_scan, summarize_point_geometry,
    transform_points_to_odom, validate_stage_lineage,
)
from salus_evaluation.static_scan_metrics import (
    StaticObstacle, StaticScanMetrics, interpolate_pose, load_obstacle_geometry,
    ray_box_intersection, scan_static_error_metrics, summarize_scan_metrics,
    summarize_pose_divergence, summarize_temporal_offset_sweep,
)


FIXTURE = (
    __import__("pathlib").Path(__file__).parents[1].parent
    / "salus_simulation" / "config" / "obstacle_drag_geometry.yaml"
)


def test_ray_box_intersection_and_perfect_scan_have_zero_error() -> None:
    obstacle = StaticObstacle("box", 4.0, 0.0, 1.0, 1.0)
    ranges = []
    for index in range(181):
        angle = -math.pi / 2.0 + index * math.pi / 180.0
        hit = ray_box_intersection(0.0, 0.0, angle, obstacle)
        ranges.append(float("inf") if hit is None else hit)
    metrics = scan_static_error_metrics(
        ranges, -math.pi / 2.0, math.pi / 180.0, Pose2D(0.0, 0.0, 0.0), [obstacle],
        range_max_m=20.0,
    )
    assert metrics.sample_count > 0
    assert metrics.scan_static_error_rmse_m == pytest.approx(0.0)
    assert metrics.scan_static_error_p95_m == pytest.approx(0.0)
    assert metrics.scored_beam_indices


def test_scan_metrics_transform_robot_pose_and_measure_error() -> None:
    obstacle = StaticObstacle("box", 5.0, 2.0, 1.0, 1.0)
    pose = Pose2D(1.0, 1.0, math.pi / 2.0)
    ranges = []
    for index in range(181):
        angle = -math.pi / 2.0 + index * math.pi / 180.0
        hit = ray_box_intersection(pose.x_m, pose.y_m, pose.yaw_rad + angle, obstacle)
        ranges.append(float("inf") if hit is None else hit + 0.2)
    metrics = scan_static_error_metrics(
        ranges, -math.pi / 2.0, math.pi / 180.0, pose, [obstacle], range_max_m=20.0,
    )
    assert metrics.sample_count > 0
    assert metrics.scan_static_error_rmse_m == pytest.approx(0.2)
    assert metrics.scan_static_error_p95_m == pytest.approx(0.2)
    assert metrics.max_error_m == pytest.approx(0.2)
    assert metrics.worst_beam_indices


def test_interpolate_pose_uses_scan_timestamp_during_motion() -> None:
    samples = [
        (10.0, Pose2D(0.0, 0.0, 0.0)),
        (12.0, Pose2D(2.0, 0.0, math.pi / 2.0)),
    ]

    at_first_half = interpolate_pose(samples, 11.0)
    at_second = interpolate_pose(samples, 11.5)

    assert at_first_half == Pose2D(1.0, 0.0, math.pi / 4.0)
    assert at_second is not None
    assert at_second.x_m == pytest.approx(1.5)
    assert at_second.yaw_rad == pytest.approx(3.0 * math.pi / 8.0)
    assert interpolate_pose(samples, 9.0) is None
    assert interpolate_pose(samples, 13.0) is None


def test_motion_scans_use_pose_at_each_scan_timestamp() -> None:
    obstacle = StaticObstacle("box", 5.0, 0.0, 1.0, 1.0)
    poses = [
        (10.0, Pose2D(0.0, 0.0, 0.0)),
        (11.0, Pose2D(1.0, 0.0, 0.0)),
    ]

    def perfect_scan(pose: Pose2D) -> list[float]:
        ranges = []
        for index in range(181):
            angle = -math.pi / 2.0 + index * math.pi / 180.0
            hit = ray_box_intersection(pose.x_m, pose.y_m, angle, obstacle)
            ranges.append(float("inf") if hit is None else hit)
        return ranges

    first_scan_pose = interpolate_pose(poses, 10.0)
    second_scan_pose = interpolate_pose(poses, 11.0)
    assert first_scan_pose is not None and second_scan_pose is not None
    assert first_scan_pose != second_scan_pose
    first_metrics = scan_static_error_metrics(
        perfect_scan(first_scan_pose), -math.pi / 2.0, math.pi / 180.0,
        first_scan_pose, [obstacle], range_max_m=20.0,
    )
    second_metrics = scan_static_error_metrics(
        perfect_scan(second_scan_pose), -math.pi / 2.0, math.pi / 180.0,
        second_scan_pose, [obstacle], range_max_m=20.0,
    )

    assert first_metrics.scan_static_error_rmse_m == pytest.approx(0.0)
    assert second_metrics.scan_static_error_rmse_m == pytest.approx(0.0)


def test_obstacle_geometry_fixture_is_versioned_and_known() -> None:
    fixed_frame, obstacles = load_obstacle_geometry(FIXTURE)
    assert fixed_frame == "odom"
    assert [obstacle.name for obstacle in obstacles] == [
        "obstacle_near_post", "obstacle_mid_wall", "obstacle_far_box",
    ]
    assert [(obstacle.x_m, obstacle.y_m) for obstacle in obstacles] == [
        (2.5, 1.5), (7.5, -2.5), (14.0, 5.0),
    ]


def test_scan_summary_perfect_control_is_zero_and_schema_is_explicit() -> None:
    summary = summarize_scan_metrics([
        (12.0, StaticScanMetrics(3, 0.0, 0.0, 0.0)),
    ])

    assert summary == {
        "samples_per_scan": [3],
        "max_scan_static_error_rmse_m": 0.0,
        "max_scan_static_error_p95_m": 0.0,
        "max_beam_static_error_m": 0.0,
        "per_scan_metrics": [{
            "stamp_s": 12.0,
            "sample_count": 3,
            "rmse_m": 0.0,
            "p95_m": 0.0,
            "max_m": 0.0,
        }],
    }


def test_scan_summary_orders_scans_and_preserves_distinct_maxima() -> None:
    summary = summarize_scan_metrics([
        (20.0, StaticScanMetrics(2, 0.4, 0.6, 0.9)),
        (10.0, StaticScanMetrics(4, 0.1, 0.2, 0.3)),
    ])

    assert summary["samples_per_scan"] == [4, 2]
    assert summary["max_scan_static_error_rmse_m"] == pytest.approx(0.4)
    assert summary["max_scan_static_error_p95_m"] == pytest.approx(0.6)
    assert summary["max_beam_static_error_m"] == pytest.approx(0.9)
    assert [entry["stamp_s"] for entry in summary["per_scan_metrics"]] == [
        10.0, 20.0,
    ]
    assert summary["per_scan_metrics"][0]["max_m"] == pytest.approx(0.3)
    assert summary["per_scan_metrics"][1]["max_m"] == pytest.approx(0.9)


def test_temporal_offset_sweep_finds_known_pose_shift() -> None:
    obstacle = StaticObstacle("box", 5.0, 0.0, 1.0, 1.0)
    poses = [
        (9.0, Pose2D(-1.0, 0.0, 0.0)),
        (11.0, Pose2D(1.0, 0.0, 0.0)),
    ]
    scan_pose = Pose2D(0.2, 0.0, 0.0)
    ranges = []
    for index in range(181):
        angle = -math.pi / 2.0 + index * math.pi / 180.0
        hit = ray_box_intersection(scan_pose.x_m, scan_pose.y_m, angle, obstacle)
        ranges.append(float("inf") if hit is None else hit)

    class ScanLike:
        angle_min = -math.pi / 2.0
        angle_increment = math.pi / 180.0
        range_min = 0.0
        range_max = 20.0

        def __init__(self, values: list[float]) -> None:
            self.ranges = values

    result = summarize_temporal_offset_sweep(
        [(10.0, ScanLike(ranges))], poses, [obstacle], [-0.2, 0.0, 0.2],
        range_max_m=20.0,
    )
    by_offset = {entry["offset_s"]: entry for entry in result}
    assert by_offset[0.2]["median_rmse_m"] == pytest.approx(0.0)
    assert by_offset[0.0]["median_rmse_m"] > 0.0
    assert by_offset[-0.2]["paired_scan_count"] == 1
    assert by_offset[0.2]["scored_beam_support"] == by_offset[0.0][
        "scored_beam_support"
    ]
    assert by_offset[0.2]["scored_beam_count"] == by_offset[0.0][
        "scored_beam_count"
    ]


def test_scan_metrics_can_pin_a_beam_support_for_comparison() -> None:
    obstacle = StaticObstacle("box", 4.0, 0.0, 1.0, 1.0)
    ranges = []
    for index in range(181):
        angle = -math.pi / 2.0 + index * math.pi / 180.0
        hit = ray_box_intersection(0.0, 0.0, angle, obstacle)
        ranges.append(float("inf") if hit is None else hit + 0.1)

    baseline = scan_static_error_metrics(
        ranges, -math.pi / 2.0, math.pi / 180.0, Pose2D(0.0, 0.0, 0.0),
        [obstacle], range_max_m=20.0,
    )
    pinned = scan_static_error_metrics(
        ranges, -math.pi / 2.0, math.pi / 180.0, Pose2D(0.0, 0.0, 0.0),
        [obstacle], range_max_m=20.0,
        beam_indices=baseline.scored_beam_indices[:2],
    )
    assert pinned.scored_beam_indices == baseline.scored_beam_indices[:2]
    assert pinned.sample_count == 2


def test_pose_divergence_has_zero_and_positive_controls() -> None:
    raw = [
        (10.0, Pose2D(0.0, 0.0, 0.0)),
        (11.0, Pose2D(1.0, 0.0, math.pi / 2.0)),
    ]
    identical = summarize_pose_divergence(raw, raw)
    divergent = summarize_pose_divergence([
        (10.0, Pose2D(0.1, 0.0, 0.0)),
        (11.0, Pose2D(1.1, 0.0, math.pi / 2.0 + 0.1)),
    ], raw)

    assert identical["status"] == "measured"
    assert identical["p95_position_error_m"] == pytest.approx(0.0)
    assert identical["p95_yaw_error_rad"] == pytest.approx(0.0)
    assert divergent["p95_position_error_m"] == pytest.approx(0.1)
    assert divergent["p95_yaw_error_rad"] == pytest.approx(0.095)


def test_pose_divergence_reports_insufficient_data_without_tf_samples() -> None:
    summary = summarize_pose_divergence([], [
        (10.0, Pose2D(0.0, 0.0, 0.0)),
        (11.0, Pose2D(1.0, 0.0, 0.0)),
    ])

    assert summary == {
        "status": "insufficient_data",
        "pose_count": 0,
        "paired_count": 0,
        "p95_position_error_m": None,
        "max_position_error_m": None,
        "p95_yaw_error_rad": None,
        "max_yaw_error_rad": None,
    }


def test_stage_point_geometry_has_zero_and_divergent_controls() -> None:
    obstacle = StaticObstacle("box", 4.0, 0.0, 1.0, 1.0)
    points = [(3.5, 0.0), (4.0, 0.5)]
    perfect = summarize_point_geometry(points, [obstacle])
    divergent = summarize_point_geometry([(3.0, 0.0)], [obstacle])
    empty = summarize_point_geometry([], [obstacle])

    assert perfect["status"] == "measured"
    assert perfect["p95_surface_error_m"] == pytest.approx(0.0)
    assert divergent["p95_surface_error_m"] == pytest.approx(0.5)
    assert empty["status"] == "insufficient_data"

    nonfinite = summarize_point_geometry(
        [(float("nan"), 0.0), (float("inf"), 0.0)], [obstacle]
    )
    assert nonfinite["status"] == "insufficient_data"

    rotated = summarize_point_geometry(
        transform_points_to_odom(points, Pose2D(0.0, 0.0, 0.1)), [obstacle]
    )
    assert rotated["status"] == "measured"
    assert rotated["p95_surface_error_m"] > 0.0


def test_stage_lineage_rejects_incomplete_stamps_and_support_mismatch() -> None:
    assert exact_common_stamps([{1, 2, 3}, {2, 3}, {3, 4}]) == (3,)
    assert exact_common_stamps([{1, 2}, {4, 5}]) == ()
    assert beam_support_is_identical([1, 2, 3], [3, 1, 2])
    assert not beam_support_is_identical([1, 2, 3], [1, 2, 4])


def test_projection_oracle_matches_nearest_point_per_bin() -> None:
    ranges = project_pointcloud_to_scan(
        [
            (2.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 1.0, 2.0),
            (float("nan"), 0.0, 0.0),
            (float("inf"), 0.0, 0.0),
        ],
        angle_min=-0.5,
        angle_max=0.5,
        angle_increment=0.5,
        range_min=0.2,
        range_max=10.0,
        min_height=-0.1,
        max_height=1.0,
    )
    assert ranges == pytest.approx((float("inf"), 1.0))


def test_projection_oracle_preserves_inclusive_boundaries_and_empty_mode() -> None:
    ranges = project_pointcloud_to_scan(
        [
            (1.0, 0.0, -1.0),
            (math.cos(1.0), math.sin(1.0), 1.0),
            (0.1, 0.0, 0.0),
        ],
        angle_min=0.0,
        angle_max=1.0,
        angle_increment=0.6,
        range_min=0.1,
        range_max=1.0,
        min_height=-1.0,
        max_height=1.0,
        use_inf=False,
        inf_epsilon=1.0,
    )
    assert ranges == pytest.approx((0.1, 1.0))
    empty = project_pointcloud_to_scan(
        [],
        angle_min=0.0,
        angle_max=1.0,
        angle_increment=0.6,
        range_min=0.1,
        range_max=1.0,
        min_height=-1.0,
        max_height=1.0,
        use_inf=False,
        inf_epsilon=1.0,
    )
    assert empty == pytest.approx((2.0, 2.0))


def test_projection_comparison_reports_zero_and_injected_divergence() -> None:
    oracle = (1.0, float("inf"), 2.0)
    equal = compare_scan_projection(oracle, (1.0 + 1.0e-6, float("inf"), 2.0))
    assert equal["finite_infinite_mismatch_count"] == 0
    assert equal["finite_agreement_count"] == 2
    assert equal["common_finite_support"] == [0, 2]
    assert equal["range_delta_m"]["max"] == pytest.approx(1.0e-6)

    divergent = compare_scan_projection(oracle, (1.5, 3.0, 2.0))
    assert divergent["finite_infinite_mismatch_count"] == 1
    assert divergent["finite_agreement_count"] == 1
    assert divergent["worst_bins"][0]["beam_index"] == 0
    assert divergent["worst_bins"][0]["abs_delta_m"] == pytest.approx(0.5)

    invalid = compare_scan_projection((float("inf"),), (float("nan"),))
    assert invalid["finite_infinite_mismatch_count"] == 1
    assert invalid["invalid_actual_count"] == 1


def _valid_stage_lineage() -> dict[str, object]:
    cloud_geometry = {
        "status": "measured", "transform_missing_count": 0,
    }
    scan_geometry = {"status": "measured", "paired_count": 10}
    return {
        "complete_chain_count": 10,
        "evaluated_chain_count": 10,
        "stamp_preserved_across_chain": True,
        "raw_to_normalized_content_equal": True,
        "scan_to_clean_metadata_equal": True,
        "common_scan_beam_count": 10,
        "projection_oracle": {
            "status": "measured",
            "paired_count": 10,
            "common_finite_support_count": 10,
            "geometry_paired_count": 10,
            "range_delta_m": {"status": "measured"},
        },
        "stages": {
            "/scan_3d_raw": {"geometry": cloud_geometry},
            "/scan_3d": {"geometry": cloud_geometry},
            "/obstacles_cloud": {"geometry": cloud_geometry},
            "/scan": {"geometry": scan_geometry},
            "/scan_clean": {"geometry": scan_geometry},
        },
    }


def test_stage_lineage_validator_rejects_invariant_mutations() -> None:
    validate_stage_lineage(_valid_stage_lineage())
    for field in (
        "stamp_preserved_across_chain",
        "raw_to_normalized_content_equal",
        "scan_to_clean_metadata_equal",
    ):
        mutated = _valid_stage_lineage()
        mutated[field] = False
        with pytest.raises(ValueError, match=field):
            validate_stage_lineage(mutated)

    missing_stage = _valid_stage_lineage()
    del missing_stage["stages"]["/scan_clean"]
    with pytest.raises(ValueError, match="stage_missing"):
        validate_stage_lineage(missing_stage)


def test_stage_lineage_validator_rejects_empty_projection_evidence() -> None:
    empty_oracle = _valid_stage_lineage()
    empty_oracle["projection_oracle"]["common_finite_support_count"] = 0
    with pytest.raises(ValueError, match="projection_oracle_support"):
        validate_stage_lineage(empty_oracle)

    unmeasured_delta = _valid_stage_lineage()
    unmeasured_delta["projection_oracle"]["range_delta_m"] = {
        "status": "insufficient_data",
    }
    with pytest.raises(ValueError, match="projection_oracle_delta"):
        validate_stage_lineage(unmeasured_delta)

    insufficient_geometry = _valid_stage_lineage()
    insufficient_geometry["stages"]["/scan_3d"]["geometry"]["status"] = (
        "insufficient_data"
    )
    with pytest.raises(ValueError, match="geometry_unmeasured"):
        validate_stage_lineage(insufficient_geometry)

    insufficient_pairs = _valid_stage_lineage()
    insufficient_pairs["stages"]["/scan"]["geometry"]["paired_count"] = 9
    with pytest.raises(ValueError, match="scan_geometry_pairs<10"):
        validate_stage_lineage(insufficient_pairs)


def test_pointcloud_signature_tracks_non_header_payload_fields() -> None:
    def message(
        is_dense: bool,
        frame_id: str,
        *,
        data: bytes = b"1234",
        point_step: int = 4,
    ) -> SimpleNamespace:
        return SimpleNamespace(
            header=SimpleNamespace(frame_id=frame_id),
            height=1,
            width=1,
            fields=(SimpleNamespace(name="x", offset=0, datatype=7, count=1),),
            is_bigendian=False,
            point_step=point_step,
            row_step=4,
            is_dense=is_dense,
            data=data,
        )

    assert pointcloud_payload_signature(message(True, "frame_a")) == (
        pointcloud_payload_signature(message(True, "frame_b"))
    )
    assert pointcloud_payload_signature(message(True, "frame_a")) != (
        pointcloud_payload_signature(message(False, "frame_a"))
    )
    assert pointcloud_payload_signature(message(True, "frame_a")) != (
        pointcloud_payload_signature(message(True, "frame_a", data=b"5678"))
    )
    assert pointcloud_payload_signature(message(True, "frame_a")) != (
        pointcloud_payload_signature(message(True, "frame_a", point_step=8))
    )


def test_stage_point_transform_uses_raw_pose() -> None:
    transformed = transform_points_to_odom(
        [(1.0, 0.0)], Pose2D(2.0, 3.0, math.pi / 2.0)
    )

    assert transformed[0][0] == pytest.approx(2.0)
    assert transformed[0][1] == pytest.approx(4.0)
