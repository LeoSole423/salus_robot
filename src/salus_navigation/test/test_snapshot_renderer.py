import pytest

from salus_navigation.snapshot_renderer import Grid, Polyline, SnapshotScene, Transform2D, render


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


def test_transform_inverse_round_trip() -> None:
    transform = Transform2D("sensor", "odom", 2.0, -1.0, 0.4)
    point = (3.0, 0.5)
    restored = transform.inverse().apply(transform.apply(point))
    assert restored[0] == pytest.approx(point[0])
    assert restored[1] == pytest.approx(point[1])
