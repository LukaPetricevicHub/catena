"""Geometry checks of docs/SPEC.md section 11. `make cad` runs them on every part.

Each check takes shapes and limits and returns numbers or findings; nothing here knows
the robot. A part is checked in the orientation it is printed in: z up, bed at its lowest z.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cadquery as cq
import numpy as np

MESH_TOLERANCE_MM = 0.02  # chord error of the triangle mesh the checks measure on
MESH_ANGLE_RAD = 0.2
AREA_PER_SAMPLE_MM2 = 1.0  # wall thickness is probed about once per this much surface
PARALLEL_DOT = -0.9  # two faces count as the sides of one wall if their normals oppose like this


@dataclass(frozen=True)
class Listed:
    """An overhang the part is allowed to have, as a box in print coordinates."""

    kind: str  # "support": printed on support material; "bridge": flat ceiling between two walls
    lo: tuple[float, float, float]
    hi: tuple[float, float, float]
    why: str
    span_mm: float = 0.0  # a bridge states how far it spans


@dataclass(frozen=True)
class Finding:
    value: float
    at: tuple[float, float, float]

    def __str__(self) -> str:
        x, y, z = self.at
        return f"{self.value:.2f} at ({x:.1f}, {y:.1f}, {z:.1f})"


def mesh(shape: cq.Shape) -> tuple[np.ndarray, np.ndarray]:
    """Vertices (n, 3) and triangles (m, 3), wound so that normals point out of the solid."""
    vertices, triangles = shape.tessellate(MESH_TOLERANCE_MM, MESH_ANGLE_RAD)
    v = np.array([(p.x, p.y, p.z) for p in vertices], dtype=float)
    t = np.array(triangles, dtype=int)
    return v, t


def _normals_areas(v: np.ndarray, t: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    cross = np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]])
    length = np.linalg.norm(cross, axis=1)
    return cross / np.maximum(length, 1e-12)[:, None], 0.5 * length


def extent(shape: cq.Shape) -> tuple[np.ndarray, np.ndarray]:
    """Lowest and highest x, y, z of the shape, from its mesh (OCC's boxes carry a margin)."""
    v, _ = mesh(shape)
    return v.min(axis=0), v.max(axis=0)


def bed_overflow_mm(shape: cq.Shape, bed_mm: tuple[float, float]) -> float:
    """How far the part's footprint exceeds the bed; 0 or less means it fits."""
    lo, hi = extent(shape)
    return float(max(hi[0] - lo[0] - bed_mm[0], hi[1] - lo[1] - bed_mm[1]))


def unlisted_overhang(
    shape: cq.Shape, max_overhang_deg: float, listed: tuple[Listed, ...] = ()
) -> Finding | None:
    """Area in mm^2 that hangs over at more than the limit and is in no listed box.

    The angle is measured from the vertical: a wall is 0, a flat ceiling 90 degrees.
    Faces lying on the bed do not count.
    """
    v, t = mesh(shape)
    normals, areas = _normals_areas(v, t)
    tri_z = v[t][:, :, 2]
    on_bed = tri_z.max(axis=1) < v[:, 2].min() + 0.01
    hanging = (-normals[:, 2] > math.sin(math.radians(max_overhang_deg)) + 1e-6) & ~on_bed
    centroids = v[t].mean(axis=1)
    for box in listed:
        lo = np.array(box.lo) - 0.05
        hi = np.array(box.hi) + 0.05
        hanging &= ~np.all((centroids >= lo) & (centroids <= hi), axis=1)
    area = float(areas[hanging].sum())
    if area < 0.05:
        return None
    worst = centroids[hanging][np.argmax(areas[hanging])]
    return Finding(area, tuple(float(c) for c in worst))


def thin_walls(shape: cq.Shape, min_wall_mm: float) -> list[Finding]:
    """Places where the material between two opposing faces is thinner than the limit.

    From sample points on the surface a ray goes into the material. If it leaves again
    through a face that looks back at the first one sooner than the limit, that is a thin
    wall. Faces at an angle to each other (chamfers, the lip of a hole) are edges, not walls.
    """
    v, t = mesh(shape)
    normals, areas = _normals_areas(v, t)
    limit = min_wall_mm - 2.5 * MESH_TOLERANCE_MM

    # Sample points, a fixed pattern per triangle so that runs repeat exactly.
    rng = np.random.default_rng(0)
    counts = np.clip(np.ceil(areas / AREA_PER_SAMPLE_MM2).astype(int), 1, 400)
    owner = np.repeat(np.arange(len(t)), counts)
    r1 = np.sqrt(rng.random(len(owner)))
    r2 = rng.random(len(owner))
    bary = np.stack([1 - r1, r1 * (1 - r2), r1 * r2], axis=1)
    points = np.einsum("ij,ijk->ik", bary, v[t[owner]])
    direction = -normals[owner]
    origin = points + direction * 1e-4

    # Triangles by grid cell; a ray only has to meet those near its start.
    cell = max(2.0 * min_wall_mm, 2.0)
    low = v.min(axis=0) - cell
    tri_lo = np.floor((v[t].min(axis=1) - low) / cell).astype(int) - 1
    tri_hi = np.floor((v[t].max(axis=1) - low) / cell).astype(int) + 1
    cells: dict[tuple[int, int, int], list[int]] = {}
    for i in range(len(t)):
        for cx in range(tri_lo[i, 0], tri_hi[i, 0] + 1):
            for cy in range(tri_lo[i, 1], tri_hi[i, 1] + 1):
                for cz in range(tri_lo[i, 2], tri_hi[i, 2] + 1):
                    cells.setdefault((cx, cy, cz), []).append(i)

    ray_cell = np.floor((origin - low) / cell).astype(int)
    order = np.lexsort((ray_cell[:, 2], ray_cell[:, 1], ray_cell[:, 0]))
    starts = np.flatnonzero(np.any(np.diff(ray_cell[order], axis=0) != 0, axis=1)) + 1
    findings: list[Finding] = []
    for group in np.split(order, starts):
        near = cells.get(tuple(int(c) for c in ray_cell[group[0]]))
        if not near:
            continue
        tri = np.array(near)
        a, b, c = (v[t[tri, k]] for k in range(3))
        o = origin[group][:, None, :]
        d = direction[group][:, None, :]
        e1 = (b - a)[None]
        e2 = (c - a)[None]
        p = np.cross(d, e2)
        det = np.einsum("ijk,ijk->ij", e1, p)
        ok = np.abs(det) > 1e-12
        inv = 1.0 / np.where(ok, det, 1.0)
        s = o - a[None]
        u = np.einsum("ijk,ijk->ij", s, p) * inv
        q = np.cross(s, e1)
        w = np.einsum("ijk,ijk->ij", d, q) * inv
        dist = np.einsum("ijk,ijk->ij", e2, q) * inv
        facing = np.einsum("ik,jk->ij", normals[owner[group]], normals[tri]) <= PARALLEL_DOT
        hit = ok & (u >= 0) & (w >= 0) & (u + w <= 1) & (dist > 1e-3) & (dist < limit) & facing
        dist = np.where(hit, dist, np.inf).min(axis=1)
        for k in np.flatnonzero(np.isfinite(dist)):
            findings.append(Finding(float(dist[k]), tuple(float(x) for x in points[group[k]])))
    findings.sort(key=lambda f: f.value)
    return findings


def overlap_mm3(a: cq.Shape, b: cq.Shape) -> float:
    """Volume two shapes share. Shapes that only touch share none."""
    ba, bb = a.BoundingBox(), b.BoundingBox()
    gap = 1e-6
    if (
        ba.xmin > bb.xmax - gap or bb.xmin > ba.xmax - gap
        or ba.ymin > bb.ymax - gap or bb.ymin > ba.ymax - gap
        or ba.zmin > bb.zmax - gap or bb.zmin > ba.zmax - gap
    ):
        return 0.0
    common = a.intersect(b)
    return float(sum(s.Volume() for s in common.Solids()))


def placed(shape: cq.Shape, yaw_deg: float = 0.0, at: tuple[float, float, float] = (0, 0, 0)):
    """The shape turned about the vertical axis through the origin, then moved."""
    out = shape
    if yaw_deg:
        out = out.rotate(cq.Vector(0, 0, 0), cq.Vector(0, 0, 1), yaw_deg)
    if any(at):
        out = out.translate(cq.Vector(*at))
    return out
