"""Offline filter with a single XYZ upload and GPU-resident intermediate arrays."""

import math

import cupy as cp
import numpy as np

from salus_perception.scan_filters import rotate_translate_points


TRANSFORM = r"""
extern "C" __global__ void transform_and_key(
    const double *source, double *points, double *radii, int *sectors, unsigned char *edges, int n,
    double qx, double qy, double qz, double qw,
    double tx, double ty, double tz, double width, double limit,
    int excluded, double full_circle
) {
    const int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= n) return;

    const double x = source[3 * i];
    const double y = source[3 * i + 1];
    const double z = source[3 * i + 2];
    const double ix = qw*x + qy*z - qz*y;
    const double iy = qw*y + qz*x - qx*z;
    const double iz = qw*z + qx*y - qy*x;
    const double iw = -qx*x - qy*y - qz*z;

    const double bx = ix*qw + iw*(-qx) + iy*(-qz) - iz*(-qy) + tx;
    const double by = iy*qw + iw*(-qy) + iz*(-qx) - ix*(-qz) + ty;
    const double bz = iz*qw + iw*(-qz) + ix*(-qy) - iy*(-qx) + tz;
    points[3*i] = bx;
    points[3*i + 1] = by;
    points[3*i + 2] = bz;
    radii[i] = 0.;
    edges[i] = 0;
    if (!isfinite(bx) || !isfinite(by) || !isfinite(bz)) {
        // Match the reference: invalid transformed points follow the obstacles
        // in their original order; finite out-of-range points are discarded.
        sectors[i] = excluded + 1;
        return;
    }

    const double radius = hypot(bx, by);
    if (radius > limit) {
        sectors[i] = excluded;
        return;
    }

    double azimuth = atan2(bx, by);
    if (azimuth < 0.) azimuth += full_circle;
    const double sector = azimuth / width;
    sectors[i] = (int)floor(sector);
    // Small absolute margin in sector units covers CPU/GPU rounding near
    // the integer boundaries without reclassifying points far from an edge.
    edges[i] = fabs(sector - nearbyint(sector)) < 1e-11;
    radii[i] = radius;
}
"""


class ResidentGroundFilter:
    def __init__(self, radial, limits, range_max):
        self.radial = radial
        self.limits = tuple(np.float64(value) for value in limits)
        self.range_max = np.float64(range_max)
        # Leave an extra sector for round-off at exactly one revolution.
        self.sector_count = math.ceil(math.tau / limits[0]) + 1
        self.sector_ids = cp.arange(self.sector_count + 1, dtype=cp.int32)
        self.transform = cp.RawKernel(TRANSFORM, "transform_and_key", options=("--fmad=false",))
        self.transform.compile()

    def device_geometry(self, source, q, t):
        n = len(source)
        xyz = cp.empty(source.shape, dtype=cp.float64)
        radii = cp.empty(n, dtype=cp.float64)
        sectors = cp.empty(n, dtype=cp.int32)
        edges = cp.empty(n, dtype=cp.uint8)
        self.transform(((n + 255) // 256,), (256,), (
            cp.asarray(source), xyz, radii, sectors, edges, np.int32(n),
            *(np.float64(value) for value in (q.x, q.y, q.z, q.w, t.x, t.y, t.z)),
            self.limits[0], self.range_max, np.int32(self.sector_count), np.float64(math.tau),
        ))
        return xyz, radii, sectors, edges

    def classify(self, source, q, t, correct_boundaries=False):
        n = len(source)
        if n == 0:
            return np.empty((0, 3), dtype=np.float32)
        xyz, radii, sectors, edges = self.device_geometry(source, q, t)
        if correct_boundaries:
            boundary_ids = cp.flatnonzero(edges)
            boundary_xyz = rotate_translate_points(
                source[cp.asnumpy(boundary_ids)], quaternion_xyzw=(q.x, q.y, q.z, q.w),
                translation_xyz=(t.x, t.y, t.z),
            )
            azimuth = np.mod(np.arctan2(boundary_xyz[:, 0], boundary_xyz[:, 1]), math.tau)
            sectors[boundary_ids] = cp.asarray(np.floor(azimuth / self.limits[0]).astype(np.int32))
        indices = cp.lexsort(cp.stack((cp.arange(n, dtype=cp.int32), radii, sectors)))
        sorted_sectors = sectors[indices]
        sorted_xyz = xyz[indices]
        sorted_radii = radii[indices]
        offsets = cp.searchsorted(sorted_sectors, self.sector_ids).astype(cp.int32)
        keep = (sorted_sectors > self.sector_count).astype(cp.uint8)
        self.radial(((self.sector_count + 127) // 128,), (128,), (
            sorted_xyz, sorted_radii, offsets, np.int32(self.sector_count),
            keep, *self.limits,
        ))
        return cp.asnumpy(sorted_xyz[keep != 0].astype(cp.float32))
