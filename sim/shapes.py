"""Strokes and target polygons (spec 7.4), written independently of core/src/shape.cpp so
that scenarios can measure how far the robots are from the shape that was drawn."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from sim.corelib import STROKE_POINTS

CLOSE_FRACTION = 0.10  # a stroke whose ends are this near, relative to its length, is closed


@dataclass(frozen=True)
class Stroke:
    """STROKE_POINTS points in pad units 0..255, y up."""

    points: tuple[tuple[int, int], ...]
    closed: bool

    def to_bytes(self) -> bytes:
        return bytes(v for p in self.points for v in p)


def _lengths(pts: np.ndarray) -> np.ndarray:
    return np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(pts, axis=0).T))])


def _along(pts: np.ndarray, count: int, edges: int) -> np.ndarray:
    """`count` points spaced evenly by arc length along a polyline of `edges` equal parts."""
    cum = _lengths(pts)
    s = cum[-1] * np.arange(count) / edges
    return np.column_stack([np.interp(s, cum, pts[:, 0]), np.interp(s, cum, pts[:, 1])])


def stroke_from_path(path: list[tuple[float, float]], closed: bool | None = None) -> Stroke:
    """Resample any drawn path (any units, y up) to a stroke that fills the pad."""
    pts = np.asarray(path, dtype=float)
    if len(pts) < 2 or _lengths(pts)[-1] <= 0.0:
        raise ValueError("a stroke needs length")
    if closed is None:
        closed = bool(np.hypot(*(pts[-1] - pts[0])) < CLOSE_FRACTION * _lengths(pts)[-1])
    if closed:
        ring = np.vstack([pts, pts[:1]])
        pts = _along(ring, STROKE_POINTS, STROKE_POINTS)
    else:
        pts = _along(pts, STROKE_POINTS, STROKE_POINTS - 1)
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    scale = 235.0 / max(float((hi - lo).max()), 1e-9)
    pts = (pts - (lo + hi) / 2) * scale + 127.5
    return Stroke(tuple((int(round(x)), int(round(y))) for x, y in pts), closed)


def named(name: str) -> Stroke:
    """The strokes behind the phone's buttons, plus S."""
    t = np.linspace(0.0, 1.0, 97)
    if name == "line":
        path = [(x, 0.0) for x in t]
    elif name == "ring":
        path = [(math.cos(2 * math.pi * a), math.sin(2 * math.pi * a)) for a in t[:-1]]
        return stroke_from_path(path, closed=True)
    elif name == "u":
        # Two legs of length 1 joined by a half circle of radius 0.5, open end up.
        path = [(-0.5, 1.0 - a) for a in t]
        path += [(-0.5 * math.cos(math.pi * a), -0.5 * math.sin(math.pi * a)) for a in t[1:]]
        path += [(0.5, a) for a in t[1:]]
    elif name == "s":
        path = [(math.cos(math.pi * (1 - a)), math.sin(math.pi * (1 - a))) for a in t]
        path += [(2.0 - math.cos(math.pi * a), -math.sin(math.pi * a)) for a in t[1:]]
    else:
        raise ValueError(f"unknown shape {name!r}")
    return stroke_from_path(path, closed=False)


def target_polygon(stroke: Stroke, count: int, pitch_mm: float) -> np.ndarray:
    """One vertex per robot, head first, mean edge one pitch, centroid at the origin."""
    pts = np.asarray(stroke.points, dtype=float)
    if stroke.closed:
        poly = _along(np.vstack([pts, pts[:1]]), count, count)
        edges = np.diff(np.vstack([poly, poly[:1]]), axis=0)
    else:
        poly = _along(pts, count, count - 1)
        edges = np.diff(poly, axis=0)
    poly = poly * pitch_mm / np.hypot(*edges.T).mean()
    return poly - poly.mean(axis=0)


@dataclass(frozen=True)
class Fit:
    mean_error_mm: float
    max_error_mm: float
    rotation_rad: float  # shape frame to world
    centroid: tuple[float, float]


def fit(positions: np.ndarray, polygon: np.ndarray) -> Fit:
    """Lay the polygon over the robots by turning and shifting it (no mirroring, no
    scaling) and report how far the robots are from their vertices."""
    centre = positions.mean(axis=0)
    p = positions - centre
    q = polygon - polygon.mean(axis=0)
    angle = math.atan2(float(np.sum(q[:, 0] * p[:, 1] - q[:, 1] * p[:, 0])),
                       float(np.sum(q[:, 0] * p[:, 0] + q[:, 1] * p[:, 1])))
    c, s = math.cos(angle), math.sin(angle)
    turned = q @ np.array([[c, s], [-s, c]])
    err = np.hypot(*(turned - p).T)
    return Fit(float(err.mean()), float(err.max()), angle, (float(centre[0]), float(centre[1])))
