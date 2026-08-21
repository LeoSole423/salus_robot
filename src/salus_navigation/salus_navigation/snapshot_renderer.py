"""Pure, deterministic renderer for navigation snapshots.

This module deliberately has no ROS imports.  The ROS adapter supplies a
coherent scene and the renderer only converts it to PNG bytes.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np


Point = Tuple[float, float]


@dataclass(frozen=True)
class Grid:
    frame_id: str
    resolution: float
    origin: Point
    width: int
    height: int
    data: Tuple[int, ...]


@dataclass(frozen=True)
class Transform2D:
    """Rigid transform mapping a point from ``source`` to ``target``."""

    source: str
    target: str
    x: float
    y: float
    yaw: float

    def apply(self, point: Point) -> Point:
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            self.x + cos_yaw * point[0] - sin_yaw * point[1],
            self.y + sin_yaw * point[0] + cos_yaw * point[1],
        )

    def inverse(self) -> "Transform2D":
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return Transform2D(
            self.target,
            self.source,
            -(cos_yaw * self.x + sin_yaw * self.y),
            sin_yaw * self.x - cos_yaw * self.y,
            -self.yaw,
        )


@dataclass(frozen=True)
class Polyline:
    frame_id: str
    points: Tuple[Point, ...]
    color_bgr: Tuple[int, int, int]
    thickness: int = 2
    closed: bool = False


@dataclass(frozen=True)
class SnapshotScene:
    local_costmap: Grid
    center_xy: Point
    extent_m: float
    size_px: int
    global_inset_px: int
    keepout: Optional[Grid] = None
    global_costmap: Optional[Grid] = None
    footprint: Optional[Polyline] = None
    stop_zone: Optional[Polyline] = None
    collision_polygons: Tuple[Polyline, ...] = ()
    scan: Optional[Polyline] = None
    plan: Optional[Polyline] = None
    transforms: Tuple[Transform2D, ...] = ()


@dataclass(frozen=True)
class RenderedSnapshot:
    png: bytes
    width: int
    height: int
    frame_id: str
    layers: Dict[str, bool]


def _lookup_transform(scene: SnapshotScene, source: str, target: str) -> Optional[Transform2D]:
    if not source or source == target:
        return Transform2D(source, target, 0.0, 0.0, 0.0)
    for transform in scene.transforms:
        if transform.source == source and transform.target == target:
            return transform
        if transform.source == target and transform.target == source:
            return transform.inverse()
    return None


def _to_frame(scene: SnapshotScene, points: Iterable[Point], source: str, target: str) -> Optional[Tuple[Point, ...]]:
    transform = _lookup_transform(scene, source, target)
    if transform is None:
        return None
    return tuple(transform.apply(point) for point in points)


def _window(scene: SnapshotScene) -> Tuple[float, float, float, float]:
    half = scene.extent_m * 0.5
    return (scene.center_xy[0] - half, scene.center_xy[0] + half,
            scene.center_xy[1] - half, scene.center_xy[1] + half)


def _world_to_px(point: Point, window: Tuple[float, float, float, float], size: int) -> Optional[Tuple[int, int]]:
    min_x, max_x, min_y, max_y = window
    x, y = point
    if x < min_x or x > max_x or y < min_y or y > max_y:
        return None
    return (
        int(round((x - min_x) / (max_x - min_x) * (size - 1))),
        int(round((max_y - y) / (max_y - min_y) * (size - 1))),
    )


def _world_to_px_unbounded(point: Point, window: Tuple[float, float, float, float], size: int) -> Tuple[int, int]:
    min_x, max_x, min_y, max_y = window
    return (
        int(round((point[0] - min_x) / (max_x - min_x) * (size - 1))),
        int(round((max_y - point[1]) / (max_y - min_y) * (size - 1))),
    )


def _grid_array(grid: Grid) -> Optional[np.ndarray]:
    if grid.width <= 0 or grid.height <= 0 or grid.resolution <= 0.0:
        return None
    if len(grid.data) != grid.width * grid.height:
        return None
    return np.asarray(grid.data, dtype=np.float32).reshape(grid.height, grid.width)[::-1, :]


def _sample_grid(scene: SnapshotScene, grid: Grid, target_frame: str, window: Tuple[float, float, float, float], border: float) -> np.ndarray:
    source = _grid_array(grid)
    size = scene.size_px
    if source is None:
        return np.full((size, size), border, dtype=np.float32)
    transform = _lookup_transform(scene, target_frame, grid.frame_id)
    if transform is None:
        return np.full((size, size), border, dtype=np.float32)
    min_x, max_x, min_y, max_y = window
    xs = np.linspace(min_x, max_x, size, dtype=np.float32)
    ys = np.linspace(max_y, min_y, size, dtype=np.float32)
    target_x, target_y = np.meshgrid(xs, ys)
    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    source_x = transform.x + cos_yaw * target_x - sin_yaw * target_y
    source_y = transform.y + sin_yaw * target_x + cos_yaw * target_y
    top_y = grid.origin[1] + grid.height * grid.resolution
    map_x = (source_x - grid.origin[0]) / grid.resolution
    map_y = (top_y - source_y) / grid.resolution
    return cv2.remap(source, map_x.astype(np.float32), map_y.astype(np.float32), cv2.INTER_NEAREST,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=float(border))


def _sample_grid_to_grid(scene: SnapshotScene, source_grid: Grid, target_grid: Grid, border: float) -> np.ndarray:
    """Sample ``source_grid`` in the exact cell geometry of ``target_grid``."""
    source = _grid_array(source_grid)
    if source is None:
        return np.full((target_grid.height, target_grid.width), border, dtype=np.float32)
    transform = _lookup_transform(scene, target_grid.frame_id, source_grid.frame_id)
    if transform is None:
        return np.full((target_grid.height, target_grid.width), border, dtype=np.float32)
    cols = np.arange(target_grid.width, dtype=np.float32)
    rows = np.arange(target_grid.height, dtype=np.float32)
    target_x, target_row = np.meshgrid(
        target_grid.origin[0] + (cols + 0.5) * target_grid.resolution,
        rows,
    )
    target_y = target_grid.origin[1] + (target_grid.height - target_row - 0.5) * target_grid.resolution
    cos_yaw = math.cos(transform.yaw)
    sin_yaw = math.sin(transform.yaw)
    source_x = transform.x + cos_yaw * target_x - sin_yaw * target_y
    source_y = transform.y + sin_yaw * target_x + cos_yaw * target_y
    source_top = source_grid.origin[1] + source_grid.height * source_grid.resolution
    map_x = (source_x - source_grid.origin[0]) / source_grid.resolution - 0.5
    map_y = (source_top - source_y) / source_grid.resolution - 0.5
    return cv2.remap(source, map_x.astype(np.float32), map_y.astype(np.float32), cv2.INTER_NEAREST,
                     borderMode=cv2.BORDER_CONSTANT, borderValue=float(border))


def _occupancy_to_color(occupancy: np.ndarray) -> np.ndarray:
    image = np.full((occupancy.shape[0], occupancy.shape[1], 3), 120, dtype=np.uint8)
    known = occupancy >= 0.0
    gray = np.clip(255.0 - np.clip(occupancy, 0.0, 100.0) * 2.3, 0.0, 255.0).astype(np.uint8)
    image[known] = np.stack([gray, gray, gray], axis=-1)[known]
    return image


def _overlay_keepout(canvas: np.ndarray, occupancy: np.ndarray) -> bool:
    cost = np.clip(occupancy, 0.0, 100.0)
    mask = cost > 0.0
    if not np.any(mask):
        return False
    alpha = np.clip(cost / 100.0, 0.05, 0.70)
    overlay = canvas.astype(np.float32).copy()
    overlay[:, :, 2] = 255.0
    overlay[:, :, :2] *= (1.0 - alpha[:, :, None] * 0.8)
    mixed = canvas.astype(np.float32)
    mixed[mask] = (1.0 - alpha[mask, None]) * mixed[mask] + alpha[mask, None] * overlay[mask]
    canvas[:, :] = np.clip(mixed, 0, 255).astype(np.uint8)
    return True


def _draw_polyline(canvas: np.ndarray, scene: SnapshotScene, line: Polyline, window: Tuple[float, float, float, float]) -> bool:
    points = _to_frame(scene, line.points, line.frame_id, scene.local_costmap.frame_id)
    if points is None or len(points) < 2:
        return False
    pixels = [_world_to_px_unbounded(point, window, scene.size_px) for point in points]
    drawn = False
    rect = (0, 0, scene.size_px, scene.size_px)
    for first, second in zip(pixels, pixels[1:]):
        visible, clipped_first, clipped_second = cv2.clipLine(rect, first, second)
        if visible:
            cv2.line(canvas, clipped_first, clipped_second, line.color_bgr, line.thickness, cv2.LINE_AA)
            drawn = True
    if line.closed and len(pixels) >= 3:
        visible, first, second = cv2.clipLine(rect, pixels[-1], pixels[0])
        if visible:
            cv2.line(canvas, first, second, line.color_bgr, line.thickness, cv2.LINE_AA)
            drawn = True
    return drawn


def _draw_points(canvas: np.ndarray, scene: SnapshotScene, line: Polyline, window: Tuple[float, float, float, float]) -> bool:
    points = _to_frame(scene, line.points, line.frame_id, scene.local_costmap.frame_id)
    if points is None:
        return False
    drawn = False
    for point in points:
        pixel = _world_to_px(point, window, scene.size_px)
        if pixel is not None:
            cv2.circle(canvas, pixel, 1, line.color_bgr, -1, cv2.LINE_AA)
            drawn = True
    return drawn


def _global_inset(canvas: np.ndarray, scene: SnapshotScene) -> bool:
    grid = scene.global_costmap
    inset_size = scene.global_inset_px
    if grid is None or inset_size <= 0 or inset_size > scene.size_px // 2:
        return False
    if _grid_array(grid) is None:
        return False
    inset_grid = Grid(
        grid.frame_id,
        max(grid.resolution, grid.width * grid.resolution / inset_size),
        grid.origin,
        inset_size,
        inset_size,
        tuple([0] * (inset_size * inset_size)),
    )
    image = _occupancy_to_color(_sample_grid_to_grid(scene, grid, inset_grid, -1.0))
    if scene.keepout is not None:
        _overlay_keepout(image, _sample_grid_to_grid(scene, scene.keepout, inset_grid, 0.0))
    if scene.plan is not None:
        plan = _to_frame(scene, scene.plan.points, scene.plan.frame_id, grid.frame_id)
        if plan is not None and len(plan) >= 2:
            window = (grid.origin[0], grid.origin[0] + grid.width * grid.resolution,
                      grid.origin[1], grid.origin[1] + grid.height * grid.resolution)
            pixels = [_world_to_px_unbounded(point, window, inset_size) for point in plan]
            for first, second in zip(pixels, pixels[1:]):
                visible, a, b = cv2.clipLine((0, 0, inset_size, inset_size), first, second)
                if visible:
                    cv2.line(image, a, b, (96, 255, 96), 2, cv2.LINE_AA)
    robot = _to_frame(scene, [scene.center_xy], scene.local_costmap.frame_id, grid.frame_id)
    if robot:
        px = _world_to_px(robot[0], (grid.origin[0], grid.origin[0] + grid.width * grid.resolution,
                                     grid.origin[1], grid.origin[1] + grid.height * grid.resolution), inset_size)
        if px is not None:
            cv2.circle(image, px, 4, (255, 0, 0), -1, cv2.LINE_AA)
    x0, y0 = scene.size_px - inset_size - 10, 10
    canvas[y0:y0 + inset_size, x0:x0 + inset_size] = image
    cv2.rectangle(canvas, (x0, y0), (x0 + inset_size, y0 + inset_size), (180, 180, 180), 1, cv2.LINE_AA)
    return True


def render(scene: SnapshotScene) -> RenderedSnapshot:
    """Render a validated scene; callers map invalid inputs to ROS errors."""
    window = _window(scene)
    canvas = _occupancy_to_color(_sample_grid(scene, scene.local_costmap, scene.local_costmap.frame_id, window, -1.0))
    layers = {name: False for name in (
        "local_costmap", "global_costmap", "keepout_mask", "footprint", "stop_zone",
        "scan", "plan", "collision_polygons", "global_inset")}
    layers["local_costmap"] = True
    if scene.keepout is not None:
        layers["keepout_mask"] = _overlay_keepout(canvas, _sample_grid(scene, scene.keepout, scene.local_costmap.frame_id, window, 0.0))
    if scene.footprint is not None:
        layers["footprint"] = _draw_polyline(canvas, scene, scene.footprint, window)
    if scene.stop_zone is not None:
        layers["stop_zone"] = _draw_polyline(canvas, scene, scene.stop_zone, window)
    for polygon in scene.collision_polygons:
        layers["collision_polygons"] = _draw_polyline(canvas, scene, polygon, window) or layers["collision_polygons"]
    if scene.scan is not None:
        layers["scan"] = _draw_points(canvas, scene, scene.scan, window)
    if scene.plan is not None:
        layers["plan"] = _draw_polyline(canvas, scene, scene.plan, window)
    if scene.global_costmap is not None:
        layers["global_inset"] = _global_inset(canvas, scene)
        layers["global_costmap"] = layers["global_inset"]
    encoded, png = cv2.imencode(".png", canvas)
    if not encoded:
        raise RuntimeError("PNG_ENCODE_FAILED: OpenCV could not encode snapshot")
    return RenderedSnapshot(png.tobytes(), scene.size_px, scene.size_px, scene.local_costmap.frame_id, layers)
