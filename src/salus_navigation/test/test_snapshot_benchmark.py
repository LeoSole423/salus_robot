"""Deterministic PC benchmark for the Nav Live snapshot payload."""

from dataclasses import dataclass
import base64
from time import perf_counter

import cv2
import numpy as np

from salus_navigation.snapshot_renderer import (
    Grid,
    KeepoutPolygon,
    Polyline,
    SnapshotScene,
    Transform2D,
    _render_canvas,
    render,
    validate_png_compression,
)


SIZES = (512, 384, 256)
COMPRESSION_LEVELS = (1, 3, 6, 9)
EXPECTED_LAYERS = {
    "local_costmap", "global_costmap", "keepout_mask", "footprint", "stop_zone",
    "scan", "plan", "collision_polygons", "global_inset",
}


@dataclass(frozen=True)
class BenchmarkResult:
    size_px: int
    compression: int
    png_bytes: int
    base64_bytes: int
    render_ms: float
    encode_ms: float


def _grid(frame_id: str, width: int, height: int, resolution: float, origin: tuple[float, float]) -> Grid:
    values = np.zeros(width * height, dtype=np.int8)
    for row in range(height):
        for column in range(width):
            if (column * 7 + row * 11) % 29 == 0:
                values[row * width + column] = 100
            elif (column + row) % 17 == 0:
                values[row * width + column] = 55
    return Grid(frame_id, resolution, origin, width, height, values)


def _scene(size_px: int) -> SnapshotScene:
    local = _grid("odom", 80, 80, 0.5, (-20.0, -20.0))
    global_grid = _grid("map", 60, 60, 1.0, (-30.0, -30.0))
    keepout = _grid("map", 60, 60, 1.0, (-30.0, -30.0))
    keepout_values = np.asarray(keepout.data).copy()
    keepout_values[28 * 60 + 28:28 * 60 + 32] = 100
    keepout_values[32 * 60 + 28:32 * 60 + 32] = 100
    keepout = Grid(
        keepout.frame_id, keepout.resolution, keepout.origin,
        keepout.width, keepout.height, keepout_values,
    )
    identity = Transform2D("map", "odom", 0.0, 0.0, 0.0)
    base_to_odom = Transform2D("base_footprint", "odom", 0.0, 0.0, 0.0)
    return SnapshotScene(
        local_costmap=local,
        center_xy=(0.0, 0.0),
        extent_m=30.0,
        size_px=size_px,
        global_inset_px=min(160, size_px // 2),
        keepout=keepout,
        global_keepout=keepout,
        vector_keepouts=(KeepoutPolygon(
            "map", ((2.0, -1.0), (4.0, -1.0), (4.0, 1.0), (2.0, 1.0)),
            transform=identity,
        ),),
        global_vector_keepouts=(KeepoutPolygon(
            "map", ((2.0, -1.0), (4.0, -1.0), (4.0, 1.0), (2.0, 1.0)),
        ),),
        global_costmap=global_grid,
        footprint=Polyline(
            "base_footprint", ((1.0, 0.4), (1.0, -0.4), (-0.2, -0.4)),
            (0, 255, 0), closed=True, transform=base_to_odom,
        ),
        stop_zone=Polyline(
            "base_footprint", ((2.0, 0.8), (2.0, -0.8), (-0.3, -0.8)),
            (0, 0, 255), closed=True, transform=base_to_odom,
        ),
        collision_polygons=(Polyline(
            "base_footprint", ((2.5, 1.0), (2.5, -1.0), (-0.2, -1.0)),
            (0, 200, 255), closed=True, transform=base_to_odom,
        ),),
        scan=Polyline(
            "base_footprint", tuple((r, r * 0.2) for r in range(2, 15)),
            (0, 80, 255), transform=base_to_odom,
        ),
        plan=Polyline(
            "map", ((-20.0, 0.0), (0.0, 0.0), (20.0, 0.0)),
            (64, 255, 64), transform=identity,
        ),
        global_plan=Polyline(
            "map", ((-20.0, 0.0), (0.0, 0.0), (20.0, 0.0)),
            (96, 255, 96),
        ),
        robot_global=(0.0, 0.0),
    )


def _render_timed(scene: SnapshotScene, compression: int) -> BenchmarkResult:
    raster_started = perf_counter()
    canvas, layers = _render_canvas(scene)
    render_ms = (perf_counter() - raster_started) * 1000.0
    encode_started = perf_counter()
    encoded, png = cv2.imencode(
        ".png", canvas, [cv2.IMWRITE_PNG_COMPRESSION, compression]
    )
    encode_ms = (perf_counter() - encode_started) * 1000.0
    assert encoded
    png_bytes = png.tobytes()
    image = cv2.imdecode(np.frombuffer(png_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
    assert image is not None
    assert image.shape[:2] == (scene.size_px, scene.size_px)
    assert layers.keys() >= EXPECTED_LAYERS
    assert all(layers.values())
    assert png_bytes.startswith(b"\x89PNG\r\n\x1a\n")
    return BenchmarkResult(
        scene.size_px,
        compression,
        len(png_bytes),
        len(base64.b64encode(png_bytes)),
        render_ms,
        encode_ms,
    )


def benchmark_matrix() -> tuple[BenchmarkResult, ...]:
    """Render a fixed all-layer scene for every requested size/level pair."""
    results = []
    for size_px in SIZES:
        scene = _scene(size_px)
        for compression in COMPRESSION_LEVELS:
            # Warm OpenCV once per size so one-time allocation does not select
            # the production profile.
            render(scene, png_compression=compression)
            samples = [_render_timed(scene, compression) for _ in range(3)]
            results.append(min(samples, key=lambda item: item.render_ms))
    return tuple(results)


def test_snapshot_configuration_and_compression_limits() -> None:
    assert validate_png_compression(0) == 0
    assert validate_png_compression(9) == 9
    for invalid in (-1, 10):
        try:
            validate_png_compression(invalid)
        except ValueError as error:
            assert "between 0 and 9" in str(error)
        else:
            raise AssertionError("invalid PNG compression was accepted")


def test_snapshot_benchmark_matrix_is_valid_and_reproducible() -> None:
    results = benchmark_matrix()
    assert len(results) == len(SIZES) * len(COMPRESSION_LEVELS)
    assert {(item.size_px, item.compression) for item in results} == {
        (size_px, compression)
        for size_px in SIZES
        for compression in COMPRESSION_LEVELS
    }
    baseline = next(item for item in results if (item.size_px, item.compression) == (512, 3))
    selected = next(item for item in results if (item.size_px, item.compression) == (384, 6))
    assert selected.png_bytes < baseline.png_bytes
    first = render(_scene(384), png_compression=6).png
    second = render(_scene(384), png_compression=6).png
    assert first == second


def _report() -> None:
    results = benchmark_matrix()
    baseline = next(item for item in results if (item.size_px, item.compression) == (512, 3))
    print("size,compression,png_bytes,base64_bytes,reduction_pct,render_ms,encode_ms,kb_s,mb_h")
    for item in results:
        reduction = 100.0 * (baseline.png_bytes - item.png_bytes) / baseline.png_bytes
        kb_s = item.base64_bytes / 1000.0
        mb_h = item.base64_bytes * 3600.0 / 1_000_000.0
        print(
            f"{item.size_px},{item.compression},{item.png_bytes},{item.base64_bytes},"
            f"{reduction:.1f},{item.render_ms:.2f},{item.encode_ms:.2f},{kb_s:.2f},{mb_h:.2f}"
        )


if __name__ == "__main__":
    _report()
