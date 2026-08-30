from array import array

import cv2
import numpy as np
import pytest

from salus_navigation.snapshot_renderer import (
    Grid,
    KeepoutPolygon,
    Polyline,
    SnapshotScene,
    Transform2D,
    _grid_array,
    _nearest_sample,
    render,
)


def _grid(frame: str = "odom") -> Grid:
    return Grid(frame, 1.0, (-2.0, -2.0), 4, 4, tuple([0] * 16))


def test_renderer_returns_png_and_layers_for_complete_scene() -> None:
    local = _grid()
    scene = SnapshotScene(
        local_costmap=local,
        center_xy=(0.0, 0.0),
        extent_m=4.0,
        size_px=128,
        global_inset_px=64,
        keepout=Grid("odom", 1.0, (-2.0, -2.0), 4, 4, tuple([100] + [0] * 15)),
        global_keepout=Grid(
            "odom", 1.0, (-2.0, -2.0), 4, 4, tuple([100] + [0] * 15),
            Transform2D("map", "odom", 0.0, 0.0, 0.0),
        ),
        global_costmap=Grid("map", 1.0, (-2.0, -2.0), 4, 4, tuple([0] * 16)),
        footprint=Polyline(
            "base_footprint", ((1.0, 0.3), (1.0, -0.3), (-0.1, -0.3)),
            (0, 255, 0), closed=True,
            transform=Transform2D("base_footprint", "odom", 0.0, 0.0, 0.0),
        ),
        stop_zone=Polyline(
            "base_footprint", ((1.5, 0.5), (1.5, -0.5), (-0.2, -0.5)),
            (0, 0, 255), closed=True,
            transform=Transform2D("base_footprint", "odom", 0.0, 0.0, 0.0),
        ),
        scan=Polyline(
            "base_footprint", ((1.0, 0.0),), (0, 80, 255),
            transform=Transform2D("base_footprint", "odom", 0.0, 0.0, 0.0),
        ),
        plan=Polyline(
            "map", ((-3.0, 0.0), (3.0, 0.0)), (64, 255, 64),
            transform=Transform2D("map", "odom", 0.0, 0.0, 0.0),
        ),
        global_plan=Polyline("map", ((-3.0, 0.0), (3.0, 0.0)), (96, 255, 96)),
        robot_global=(0.0, 0.0),
    )
    output = render(scene)
    assert output.png.startswith(b"\x89PNG\r\n\x1a\n")
    assert output.width == output.height == 128
    assert output.layers == {
        "local_costmap": True, "global_costmap": True, "keepout_mask": True,
        "footprint": True, "stop_zone": True, "scan": True, "plan": True,
        "collision_polygons": False, "global_inset": True,
    }


def test_renderer_omits_optional_layer_when_transform_is_missing() -> None:
    output = render(SnapshotScene(
        local_costmap=_grid(), center_xy=(0.0, 0.0), extent_m=4.0,
        size_px=128, global_inset_px=64,
        plan=Polyline("map", ((-1.0, 0.0), (1.0, 0.0)), (64, 255, 64)),
    ))
    assert output.layers["local_costmap"] is True
    assert output.layers["plan"] is False


def test_renderer_draws_vector_keepout_with_a_hole_without_global_raster() -> None:
    output = render(SnapshotScene(
        local_costmap=_grid(), center_xy=(0.0, 0.0), extent_m=4.0,
        size_px=128, global_inset_px=64,
        vector_keepouts=(KeepoutPolygon(
            "map", ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)),
            (((-0.3, -0.3), (0.3, -0.3), (0.3, 0.3), (-0.3, 0.3)),),
            Transform2D("map", "odom", 0.0, 0.0, 0.0),
        ),),
    ))
    assert output.layers["keepout_mask"] is True


def test_transform_inverse_round_trip() -> None:
    transform = Transform2D("sensor", "odom", 2.0, -1.0, 0.4)
    point = (3.0, 0.5)
    restored = transform.inverse().apply(transform.apply(point))
    assert restored[0] == pytest.approx(point[0])
    assert restored[1] == pytest.approx(point[1])



def test_grid_array_reuses_signed_byte_backing_storage() -> None:
    values = array("b", [-1, 0, 25, 100])
    grid = Grid("map", 1.0, (0.0, 0.0), 2, 2, values)
    source = _grid_array(grid)

    assert source is not None
    assert source.dtype == np.int8
    assert np.shares_memory(source, np.frombuffer(values, dtype=np.int8))
    assert source.tolist() == [[25, 100], [-1, 0]]


def test_nearest_sample_matches_opencv_reference_without_full_float_source() -> None:
    source = np.arange(25, dtype=np.int8).reshape(5, 5)
    map_x = np.array([
        [-0.6, -0.5, -0.49, 0.0, 0.49, 0.5, 0.51],
        [1.49, 1.5, 1.51, 3.49, 3.5, 4.49, 4.6],
    ], dtype=np.float32)
    map_y = np.array([
        [0.0] * 7,
        [4.0] * 7,
    ], dtype=np.float32)
    border = -9.0

    expected = cv2.remap(
        source.astype(np.float32),
        map_x,
        map_y,
        cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=border,
    )
    sampled = _nearest_sample(source, map_x, map_y, border)

    np.testing.assert_array_equal(sampled, expected)
    assert sampled.dtype == np.float32
