"""Target-first solver for a two-arc pocket with a terminal stick.

Vendored from ``/home/user/ws/grasp/hug/hug.py`` for isolated Pilot releases.
Only its static contact solver is retained; GUI/playback code is excluded.

The target is a fixed-size uniform simple polygon.  A solution chooses the
common physical arc length and rigid pose; it never resizes the target.
Candidate grasps are constructed from target contact pairs instead of sampling
complete gripper poses.
"""

# ============================================================================
# REFERENCE / DESIGN SOURCE LINKS (hug; forked from grasp004)
# ----------------------------------------------------------------------------
# Local project lineage:
#   - newarc.py
#   - grasp001.py / ADD001.md
#   - grasp002.py / ADD002.md
#   - AGENTS.md
#   - Aug071413.md
#   - PLANS.md (PLANS(1).md in the source bundle)
#
# Continuum / contact-aware motion-planning references retained from the
# previous prototypes:
#   https://github.com/ContinuumRoboticsLab/OpenTDCRContactModel
#   https://arxiv.org/abs/2402.14175
#   https://arxiv.org/abs/1812.03615
#   https://pmc.ncbi.nlm.nih.gov/articles/PMC9752401/
#   https://github.com/ompl/ompl
#
# hug delta:
#   - inherits grasp003's fixed-heading rail planner and rectangular body
#   - uses canonical target pose (0,0,0) in the normal GUI/CLI path
#   - GUI saves only the visible preview canvas framebuffer as GIF/video
#   - headless CLI saves playback through the Matplotlib writer
#   - explicit static-solution selection before path generation and playback
#   - q=(rail slide, turn1, turn2), with fixed robot heading per plan
#   - user-configured initial turn1/turn2 good-start search
#   - rectangular robot body attached behind the arc base and collision-checked
# ============================================================================

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, pi, sin, sqrt
import time
from typing import Callable, Iterable

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.optimize import brentq, minimize_scalar
from shapely import contains_xy
from shapely.geometry import (
    GeometryCollection,
    LineString,
    MultiLineString,
    MultiPoint,
    Point,
    Polygon,
)
from shapely.geometry.polygon import orient
from shapely.ops import nearest_points, polygonize, substring, triangulate, unary_union
from shapely.validation import explain_validity

Array = np.ndarray
Progress = Callable[[dict], None]

ARC1_COLOR = (1.0, 0.50, 0.05)
ARC2_COLOR = (0.17, 0.63, 0.17)
STICK_COLOR = (0.66, 0.88, 0.60)
TARGET_COLOR = (0.58, 0.40, 0.74)
TEMPLATES = ("custom", "circle", "triangle", "square", "pentagon", "hexagon")
TEMPLATE_LABELS = ("Custom", "Circle", "Triangle", "Square", "Pentagon", "Hexagon")
DEFAULT_MOTION_MAX_TURN_RATE_DEG = 30.0
DEFAULT_MOTION_MAX_SLIDE_SPEED = 0.40


class DesignError(ValueError):
    pass


@dataclass(frozen=True)
class DistanceField:
    values: Array
    low: Array
    step: float

    def query(self, points: Array) -> Array:
        """Bilinearly interpolate signed distance; negative means inside."""
        points = np.atleast_2d(np.asarray(points, dtype=float))
        uv = (points - self.low) / self.step
        x, y = uv[:, 0], uv[:, 1]
        height, width = self.values.shape
        outside = (x < 0.0) | (x > width - 1) | (y < 0.0) | (y > height - 1)
        x = np.clip(x, 0.0, width - 1.000001)
        y = np.clip(y, 0.0, height - 1.000001)
        x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
        x1, y1 = np.minimum(x0 + 1, width - 1), np.minimum(y0 + 1, height - 1)
        fx, fy = x - x0, y - y0
        values = (
            (1.0 - fx) * (1.0 - fy) * self.values[y0, x0]
            + fx * (1.0 - fy) * self.values[y0, x1]
            + (1.0 - fx) * fy * self.values[y1, x0]
            + fx * fy * self.values[y1, x1]
        )
        if np.any(outside):
            values[outside] = np.maximum(values[outside], self.step)
        return values


@dataclass(frozen=True)
class Target:
    vertices: Array
    polygon: Polygon
    centroid: Array
    area: float
    perimeter: float
    major_axis: float
    distance_field: DistanceField
    triangles: tuple[Array, ...]


@dataclass(frozen=True)
class ContactFeature:
    point: Array
    tangent: float
    normal: Array
    u: float
    vertex: bool


@dataclass(frozen=True)
class Gripper:
    turn1: float
    turn2: float
    n: float
    length: float
    arc1: Array
    arc2: Array
    stick: Array
    side2: Array
    e1: Array
    q: Array
    e2: Array
    tip: Array
    region: Polygon | None


@dataclass(frozen=True)
class Candidate:
    mode: str
    turn1: float
    turn2: float
    a: float
    second: float
    length: float
    rotation: float
    translation: Array
    local_p1: Array
    local_p2: Array
    tangent1: float
    tangent2: float
    u1: float
    u2: float
    winding: int
    coarse_penetration: float = 0.0
    coarse_capture_gap: float = 0.0


@dataclass(frozen=True)
class Solution:
    gripper: Gripper
    target: Polygon
    mode: str
    length: float
    rotation: float
    translation: Array
    p1: Array
    p2: Array
    a: float
    second: float
    u1: float
    u2: float
    capture: Polygon
    depth_score: float
    p1_score: float
    score: float
    feasible: bool
    violations: tuple[str, ...] = ()


@dataclass(frozen=True)
class Settings:
    n: float = 0.155
    solution_count: int = 5
    time_limit: float = 10.0
    wait_for_first: bool = False
    boundary_levels: tuple[int, ...] = (32, 64, 128)
    sdf_resolution: int = 256
    contact_guard: float = 0.03
    contact_separation: float = 0.02
    pair_limits: tuple[int, ...] = (128, 512, 2048)
    contact_grid: int = 5
    turn_samples: int = 33
    refinement_levels: int = 3
    exact_tolerance: float = 1.0e-5
    min_contact_separation_relative: float = 1.0e-4
    min_capture_area_relative: float = 1.0e-6
    centroid_margin_relative: float = 1.0e-4

    def checked(self) -> "Settings":
        if not 0.0 <= self.n <= 1.0:
            raise DesignError("gripper ratio must be between 0 and 1")
        if self.solution_count < 1:
            raise DesignError("solution count must be at least 1")
        if self.time_limit <= 0.0:
            raise DesignError("time limit must be positive")
        if self.sdf_resolution < 32:
            raise DesignError("SDF resolution must be at least 32")
        if self.contact_grid < 3 or self.turn_samples < 9:
            raise DesignError("search grids are too small")
        if self.exact_tolerance <= 0.0:
            raise DesignError("exact tolerance must be positive")
        for name in (
            "min_contact_separation_relative",
            "min_capture_area_relative",
            "centroid_margin_relative",
        ):
            if getattr(self, name) < 0.0:
                raise DesignError(f"{name} must be non-negative")
        return self


def rot90(v: Array) -> Array:
    return np.array([-v[1], v[0]], dtype=float)


def rotate(v: Array, angle: float) -> Array:
    values = np.asarray(v, dtype=float)
    c, s = cos(angle), sin(angle)
    matrix = np.array([[c, -s], [s, c]])
    return values @ matrix.T


def cross2(a: Array, b: Array) -> float:
    return float(a[0] * b[1] - a[1] * b[0])


def wrap_pi(angle: float) -> float:
    return float((angle + pi) % (2.0 * pi) - pi)


def wrap_line(angle: float) -> float:
    """Wrap an unoriented line angle to [-pi/2, pi/2)."""
    return float((angle + pi / 2.0) % pi - pi / 2.0)


def arc_point(start: Array, tangent: Array, curvature: float, s) -> Array:
    values = np.atleast_1d(np.asarray(s, dtype=float))
    if abs(curvature) < 1.0e-12:
        points = start + values[:, None] * tangent
    else:
        z = curvature * values
        points = start + (
            np.sin(z)[:, None] * tangent
            + (1.0 - np.cos(z))[:, None] * rot90(tangent)
        ) / curvature
    return points[0] if np.asarray(s).ndim == 0 else points


def _clean_vertices(vertices: Iterable[Iterable[float]]) -> Array:
    points = np.asarray(tuple(vertices), dtype=float)
    if points.ndim != 2 or points.shape[1:] != (2,):
        raise DesignError("vertices must be an N by 2 coordinate array")
    if len(points) > 1 and np.linalg.norm(points[0] - points[-1]) <= 1.0e-12:
        points = points[:-1]
    if len(points) < 3 or not np.isfinite(points).all():
        raise DesignError("a finite polygon needs at least three vertices")

    span = max(float(np.ptp(points[:, 0])), float(np.ptp(points[:, 1])), 1.0)
    tolerance = 1.0e-9 * span
    kept = [points[0]]
    for point in points[1:]:
        if np.linalg.norm(point - kept[-1]) > tolerance:
            kept.append(point)
    points = np.asarray(kept)
    if len(points) > 2 and np.linalg.norm(points[0] - points[-1]) <= tolerance:
        points = points[:-1]

    changed = True
    while changed and len(points) > 3:
        changed = False
        keep = np.ones(len(points), dtype=bool)
        for i in range(len(points)):
            before, here, after = points[i - 1], points[i], points[(i + 1) % len(points)]
            left, right = here - before, after - here
            scale = max(np.linalg.norm(left) * np.linalg.norm(right), tolerance)
            if abs(cross2(left, right)) <= tolerance * scale and float(left @ right) >= 0.0:
                keep[i] = False
                changed = True
        if int(np.sum(keep)) < 3:
            break
        points = points[keep]
    if len(points) < 3:
        raise DesignError("the polygon collapses after duplicate points are removed")
    return points


def major_axis_length(vertices: Iterable[Iterable[float]]) -> float:
    """Return the longest vertex-to-vertex distance of a polygon."""
    points = _clean_vertices(vertices)
    differences = points[:, None, :] - points[None, :, :]
    return float(np.sqrt(np.max(np.sum(differences * differences, axis=2))))


def resize_major_axis(vertices: Iterable[Iterable[float]], length: float) -> Array:
    """Center a polygon at its uniform centroid and set its longest diagonal."""
    if not np.isfinite(length) or length <= 0.0:
        raise DesignError("major axis must be positive")
    points = _clean_vertices(vertices)
    polygon = Polygon(points)
    if not polygon.is_valid or polygon.area <= 1.0e-12:
        reason = explain_validity(polygon) if not polygon.is_valid else "zero area"
        raise DesignError(f"invalid polygon: {reason}")
    center = np.asarray(polygon.centroid.coords[0], dtype=float)
    diameter = major_axis_length(points)
    if diameter <= 1.0e-12:
        raise DesignError("major axis must be positive")
    return (points - center) * (float(length) / diameter)


def template_vertices(kind: str, size: float, *, circle_vertices: int = 64) -> Array:
    """Create a centered circle or regular polygon with the requested size.

    ``size`` is the radius for a circle and the side length for regular
    polygons.
    """
    name = str(kind).strip().lower()
    if name not in TEMPLATES[1:]:
        raise DesignError(f"unknown target template: {kind}")
    if not np.isfinite(size) or size <= 0.0:
        raise DesignError("template size must be positive")
    if name == "circle":
        sides = max(int(circle_vertices), 16)
        radius = float(size)
    else:
        sides = {"triangle": 3, "square": 4, "pentagon": 5, "hexagon": 6}[name]
        radius = float(size) / (2.0 * sin(pi / sides))
    phase = pi / 2.0 if sides % 2 else pi / 2.0 - pi / sides
    angles = phase + np.arange(sides) * 2.0 * pi / sides
    return np.column_stack([radius * np.cos(angles), radius * np.sin(angles)])


def _distance_field(poly: Polygon, resolution: int) -> DistanceField:
    min_x, min_y, max_x, max_y = poly.bounds
    width, height = max_x - min_x, max_y - min_y
    side = max(width, height, 1.0e-9)
    center = np.array([(min_x + max_x) / 2.0, (min_y + max_y) / 2.0])
    half = 0.75 * side
    low = center - half
    high = center + half
    coordinates = np.linspace(low[0], high[0], resolution)
    vertical = np.linspace(low[1], high[1], resolution)
    x, y = np.meshgrid(coordinates, vertical)
    inside = np.asarray(contains_xy(poly, x, y), dtype=bool)
    step = float((high[0] - low[0]) / (resolution - 1))
    inside_distance = distance_transform_edt(inside) * step
    outside_distance = distance_transform_edt(~inside) * step
    signed = np.where(inside, -inside_distance, outside_distance)
    return DistanceField(signed, low, step)


def _mesh(poly: Polygon) -> tuple[Array, ...]:
    result = []
    for triangle in triangulate(poly):
        if poly.buffer(1.0e-12).covers(triangle):
            result.append(np.asarray(triangle.exterior.coords[:3], dtype=float))
    return tuple(result)


def make_target(vertices: Iterable[Iterable[float]], *, sdf_resolution: int = 256) -> Target:
    """Validate and center a fixed-size uniform simple polygon."""
    points = _clean_vertices(vertices)
    raw = Polygon(points)
    if not raw.is_valid:
        raise DesignError(f"invalid polygon: {explain_validity(raw)}")
    if raw.area <= 1.0e-12:
        raise DesignError("the polygon area must be positive")
    raw = orient(raw, 1.0)
    center = np.asarray(raw.centroid.coords[0], dtype=float)
    centered = np.asarray(raw.exterior.coords[:-1]) - center
    poly = orient(Polygon(centered), 1.0)
    centered = np.asarray(poly.exterior.coords[:-1], dtype=float)
    major_axis = major_axis_length(centered)
    return Target(
        centered,
        poly,
        np.asarray(poly.centroid.coords[0], dtype=float),
        float(poly.area),
        float(poly.length),
        major_axis,
        _distance_field(poly, sdf_resolution),
        _mesh(poly),
    )


def make_custom_target(
    vertices: Iterable[Iterable[float]], major_axis: float, *, sdf_resolution: int = 256
) -> Target:
    return make_target(
        resize_major_axis(vertices, major_axis), sdf_resolution=sdf_resolution
    )


def make_template_target(
    kind: str, size: float, *, sdf_resolution: int = 256, circle_vertices: int = 64
) -> Target:
    return make_target(
        template_vertices(kind, size, circle_vertices=circle_vertices),
        sdf_resolution=sdf_resolution,
    )


def boundary_features(target: Target, count: int, *, cone_step=5.0) -> tuple[ContactFeature, ...]:
    """Sample edges and polygon vertex tangent cones in normalized arclength."""
    vertices = target.vertices
    edges = np.roll(vertices, -1, axis=0) - vertices
    lengths = np.linalg.norm(edges, axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(lengths)])
    perimeter = float(cumulative[-1])
    result: list[ContactFeature] = []

    def add(point, angle, u, vertex):
        tangent = np.array([cos(angle), sin(angle)])
        normal = np.array([tangent[1], -tangent[0]])
        probe_size = max(1.0e-10, 1.0e-5 * target.major_axis)
        probe = np.asarray(point, dtype=float) + probe_size * normal
        if target.polygon.covers(Point(*probe)):
            angle += pi
            tangent = -tangent
            normal = -normal
        result.append(
            ContactFeature(np.asarray(point, dtype=float), float(angle), normal, float(u % 1.0), vertex)
        )

    for distance in np.linspace(0.0, perimeter, count, endpoint=False):
        index = min(int(np.searchsorted(cumulative, distance, side="right") - 1), len(edges) - 1)
        fraction = (distance - cumulative[index]) / lengths[index]
        point = vertices[index] + fraction * edges[index]
        add(point, atan2(edges[index, 1], edges[index, 0]), distance / perimeter, False)

    max_step = np.radians(max(float(cone_step), 0.5))
    for i, point in enumerate(vertices):
        before = edges[i - 1]
        after = edges[i]
        first = atan2(before[1], before[0])
        second = atan2(after[1], after[0])
        # For a CCW polygon, the directed incoming-to-outgoing turn traces the
        # outward normal cone.  Using the shorter unoriented line interval
        # would choose the wrong side at convex corners such as a triangle.
        delta = wrap_pi(second - first)
        steps = max(1, int(np.ceil(abs(delta) / max_step)))
        angles = [first + delta * j / steps for j in range(steps + 1)]
        angles.append(first + delta / 2.0)
        u = cumulative[i] / perimeter
        for angle in angles:
            add(point, angle, u, True)

    unique: dict[tuple[float, float, float], ContactFeature] = {}
    for item in result:
        key = (round(item.point[0], 8), round(item.point[1], 8), round(wrap_pi(item.tangent), 6))
        unique[key] = item
    return tuple(unique.values())


def _finite_region(chain: Array) -> Polygon | None:
    if len(chain) < 3:
        return None
    direct = Polygon(chain)
    boundary = LineString(np.vstack([chain, chain[0]]))
    if boundary.is_simple and direct.is_valid and direct.area > 1.0e-9:
        return orient(direct, 1.0)
    regions = [item for item in polygonize(unary_union(boundary)) if item.area > 1.0e-9]
    return orient(max(regions, key=lambda item: item.area), 1.0) if regions else None


def _adaptive_samples(turn: float, tolerance: float, length: float = 1.0) -> int:
    if abs(turn) <= 1.0e-12:
        return 2
    radius = length / abs(turn)
    ratio = min(max(tolerance / radius, 1.0e-12), 1.999999)
    step = 2.0 * acos(1.0 - ratio)
    return max(9, min(721, int(np.ceil(abs(turn) / step)) + 1))


def build_gripper(
    turn1: float,
    turn2: float,
    n: float = 0.155,
    *,
    length: float = 1.0,
    samples: int | tuple[int, int] = 81,
    with_region: bool = True,
    extra1: Iterable[float] = (),
    extra2: Iterable[float] = (),
    extra_stick: Iterable[float] = (),
) -> Gripper:
    if not np.isfinite([turn1, turn2, n, length]).all():
        raise DesignError("gripper parameters must be finite")
    if abs(turn1) > pi + 1.0e-10 or abs(turn2) > pi + 1.0e-10:
        raise DesignError("arc turns must be between -180 and 180 degrees")
    if not 0.0 <= n <= 1.0:
        raise DesignError("gripper ratio must be between 0 and 1")
    if length <= 0.0:
        raise DesignError("arc length must be positive")
    sample1, sample2 = (samples, samples) if isinstance(samples, int) else samples
    values1 = np.unique(np.clip(np.r_[np.linspace(0.0, 1.0, sample1), tuple(extra1)], 0.0, 1.0))
    values2 = np.unique(np.clip(np.r_[np.linspace(0.0, 1.0, sample2), tuple(extra2)], 0.0, 1.0))
    e1 = np.zeros(2)
    tangent = np.array([0.0, 1.0])
    arc1 = arc_point(e1, tangent, turn1, values1)
    q = arc_point(e1, tangent, turn1, 1.0)
    tangent2 = rotate(tangent, turn1)
    arc2 = arc_point(q, tangent2, turn2, values2)
    e2 = arc_point(q, tangent2, turn2, 1.0)
    terminal = rotate(tangent, turn1 + turn2)
    tip = e2 + n * terminal
    if n > 1.0e-12:
        stick_count = max(2, int(np.ceil(max(sample1, sample2) * n)) + 1)
        stick_values = np.unique(np.clip(np.r_[np.linspace(0.0, n, stick_count), tuple(extra_stick)], 0.0, n))
        stick = e2 + stick_values[:, None] * terminal
    else:
        stick = e2[None, :]
    arc1[0], arc1[-1], arc2[0], arc2[-1] = e1, q, q, e2
    if len(stick):
        stick[0], stick[-1] = e2, tip
    side2 = np.vstack([arc2, stick[1:]])
    chain = np.vstack([arc1, side2[1:]])
    length = float(length)
    arc1 *= length
    arc2 *= length
    stick *= length
    side2 *= length
    e1 *= length
    q *= length
    e2 *= length
    tip *= length
    chain *= length
    return Gripper(
        float(turn1),
        float(turn2),
        float(n),
        length,
        arc1,
        arc2,
        stick,
        side2,
        e1,
        q,
        e2,
        tip,
        _finite_region(chain) if with_region else None,
    )


@dataclass(frozen=True)
class _Pair:
    first: ContactFeature
    second: ContactFeature
    priority: float


def _point_segment_distance(point: Array, start: Array, end: Array) -> float:
    edge = end - start
    squared = float(edge @ edge)
    if squared <= 1.0e-20:
        return float(np.linalg.norm(point - start))
    fraction = float(np.clip((point - start) @ edge / squared, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + fraction * edge)))


def _cyclic_distance(left: float, right: float) -> float:
    difference = abs(left - right) % 1.0
    return min(difference, 1.0 - difference)


def _contact_pairs(
    features: tuple[ContactFeature, ...],
    settings: Settings,
    target: Target,
    should_stop: Callable[[], bool] | None = None,
) -> list[_Pair]:
    """Build ordered contact pairs without an O(N²) Python loop.

    The old implementation evaluated every pair through Python helpers.  For
    the default circle that meant 35,136 Python iterations before the first
    candidate could even be tested, which made a one-second solver budget
    expire during preprocessing.  All pair scores are independent, so do the
    numerical part in NumPy and only materialize the retained pairs.

    The row-major mask order and stable sort intentionally match the old
    nested-loop order for equal priorities.
    """
    if should_stop is not None and should_stop():
        return []
    if not features:
        return []
    points = np.asarray([item.point for item in features], dtype=float)
    normals = np.asarray([item.normal for item in features], dtype=float)
    positions = np.asarray([item.u for item in features], dtype=float)
    size = max(target.major_axis, 1.0e-12)
    start = points[:, None, :]
    chord = points[None, :, :] - start
    length_sq = np.einsum("ijk,ijk->ij", chord, chord)
    length = np.sqrt(length_sq)

    separation = np.abs(positions[:, None] - positions[None, :])
    separation = np.minimum(separation, 1.0 - separation)
    valid = (separation >= settings.contact_separation) & (length >= 1.0e-3 * size)

    safe_length_sq = np.maximum(length_sq, 1.0e-20)
    fraction = -np.einsum("ijk,ijk->ij", start, chord) / safe_length_sq
    fraction = np.clip(fraction, 0.0, 1.0)
    closest = start + fraction[..., None] * chord
    depth = np.linalg.norm(closest, axis=2)

    normal_dot = normals @ normals.T
    opposition = np.clip((1.0 - normal_dot) / 2.0, 0.0, 1.0)
    priority = (
        0.20 * depth / (depth + size)
        + 0.65 * opposition
        + 0.15 * np.minimum(length / size, 1.0)
    )

    first_indices, second_indices = np.nonzero(valid)
    if should_stop is not None and should_stop():
        return []
    if len(first_indices) == 0:
        return []
    priorities = priority[first_indices, second_indices]
    order = np.argsort(-priorities, kind="stable")
    return [
        _Pair(
            features[int(first_indices[index])],
            features[int(second_indices[index])],
            float(priorities[index]),
        )
        for index in order
    ]


def _select_pairs(
    pairs: list[_Pair],
    limit: int,
    bins=16,
    should_stop: Callable[[], bool] | None = None,
) -> list[_Pair]:
    """Keep high-priority pairs without starving any boundary-to-boundary region."""
    if should_stop is not None and should_stop():
        return []
    if len(pairs) <= limit:
        return pairs
    selected: list[_Pair] = []
    used: set[int] = set()

    # Preserve tangent-cone variants together.  Keeping only the best tangent
    # for each spatial pair can discard the one line that avoids penetration.
    spatial: dict[tuple[float, float], list[_Pair]] = {}
    for pair in pairs:
        spatial.setdefault((round(pair.first.u, 6), round(pair.second.u, 6)), []).append(pair)
    groups = sorted(
        spatial.values(),
        key=lambda group: (
            min(float(item.first.normal @ item.second.normal) for item in group),
            -float(np.linalg.norm(group[0].second.point - group[0].first.point)),
            abs(_cyclic_distance(group[0].first.u, group[0].second.u) - 0.5),
            -max(item.priority for item in group),
        ),
    )
    reserve = max(1, limit // 2)
    for group in groups:
        if should_stop is not None and should_stop():
            return []
        variants = sorted(
            group,
            key=lambda item: (float(item.first.normal @ item.second.normal), -item.priority),
        )
        for pair in variants[:12]:
            if len(selected) >= reserve:
                break
            selected.append(pair)
            used.add(id(pair))
        if len(selected) >= reserve:
            break

    buckets: dict[tuple[int, int], _Pair] = {}
    for pair in pairs:
        key = (
            min(int(pair.first.u * bins), bins - 1),
            min(int(pair.second.u * bins), bins - 1),
        )
        buckets.setdefault(key, pair)
    diverse = sorted(buckets.values(), key=lambda item: item.priority, reverse=True)
    room = max(0, 3 * limit // 4 - len(selected))
    if len(diverse) > room and room:
        stride = len(diverse) / room
        diverse = [diverse[min(int(index * stride), len(diverse) - 1)] for index in range(room)]
    for pair in diverse:
        if should_stop is not None and should_stop():
            return []
        if len(selected) >= 3 * limit // 4:
            break
        if id(pair) not in used:
            selected.append(pair)
            used.add(id(pair))
    for pair in pairs:
        if should_stop is not None and should_stop():
            return []
        if len(selected) >= limit:
            break
        if id(pair) not in used:
            selected.append(pair)
            used.add(id(pair))
    return selected


def _contact_geometry(
    turn1: float,
    turn2: float,
    a: float,
    second: float,
    mode: str,
    length: float = 1.0,
) -> tuple[Array, Array, float, float]:
    tangent0 = np.array([0.0, 1.0])
    e1 = np.zeros(2)
    p1 = arc_point(e1, tangent0, turn1, a)
    tangent1 = pi / 2.0 + a * turn1
    q = arc_point(e1, tangent0, turn1, 1.0)
    tangent_q = rotate(tangent0, turn1)
    if mode == "arc2":
        p2 = arc_point(q, tangent_q, turn2, second)
        tangent2 = pi / 2.0 + turn1 + second * turn2
    elif mode == "stick":
        e2 = arc_point(q, tangent_q, turn2, 1.0)
        terminal = rotate(tangent0, turn1 + turn2)
        p2 = e2 + second * terminal
        tangent2 = pi / 2.0 + turn1 + turn2
    else:
        raise DesignError(f"unknown contact mode: {mode}")
    return length * p1, length * p2, tangent1, tangent2


def _canonical_arc_xy(turn: float, distance: float) -> tuple[float, float]:
    """Arc displacement for a +y initial tangent without allocating arrays."""
    if abs(turn) < 1.0e-12:
        return 0.0, float(distance)
    angle = turn * distance
    return -(1.0 - cos(angle)) / turn, sin(angle) / turn


def _contact_geometry_xy(
    turn1: float, turn2: float, a: float, second: float, mode: str
) -> tuple[float, float, float, float, float]:
    p1x, p1y = _canonical_arc_xy(turn1, a)
    qx, qy = _canonical_arc_xy(turn1, 1.0)
    local_x, local_y = _canonical_arc_xy(turn2, second if mode == "arc2" else 1.0)
    c1, s1 = cos(turn1), sin(turn1)
    p2x = qx + c1 * local_x - s1 * local_y
    p2y = qy + s1 * local_x + c1 * local_y
    if mode == "stick":
        p2x -= second * sin(turn1 + turn2)
        p2y += second * cos(turn1 + turn2)
    return p1x, p1y, p2x, p2y, pi / 2.0 + a * turn1


def _turn2(
    delta: float, winding: int, turn1: float, a: float, second: float, mode: str
) -> float | None:
    target = delta + winding * pi
    if mode == "arc2":
        if second <= 1.0e-12:
            return None
        value = (target - (1.0 - a) * turn1) / second
    else:
        value = target - (1.0 - a) * turn1
    return float(value) if abs(value) <= pi + 1.0e-10 else None


def _construct_candidate(
    pair: _Pair,
    mode: str,
    a: float,
    second: float,
    turn1: float,
    winding: int,
) -> Candidate | None:
    delta = wrap_line(pair.second.tangent - pair.first.tangent)
    turn2 = _turn2(delta, winding, turn1, a, second, mode)
    if turn2 is None:
        return None
    g1, g2, tangent1, tangent2 = _contact_geometry(turn1, turn2, a, second, mode)
    local_chord = pair.second.point - pair.first.point
    rotation = tangent1 - pair.first.tangent
    transformed_chord = rotate(local_chord, rotation)
    denominator = float(transformed_chord @ transformed_chord)
    gripper_chord = g2 - g1
    if denominator <= 1.0e-14 or np.linalg.norm(gripper_chord) <= 1.0e-9:
        return None
    inverse_length = float(gripper_chord @ transformed_chord / denominator)
    if inverse_length < 0.0:
        rotation += pi
        transformed_chord = -transformed_chord
        inverse_length = -inverse_length
    if not np.isfinite(inverse_length) or inverse_length <= 1.0e-12:
        return None
    residual = abs(cross2(gripper_chord, transformed_chord))
    residual /= max(np.linalg.norm(gripper_chord) * np.linalg.norm(transformed_chord), 1.0e-14)
    if residual > 2.0e-6:
        return None
    length = 1.0 / inverse_length
    if not np.isfinite(length) or length <= 1.0e-12:
        return None
    translation = length * g1 - rotate(pair.first.point, rotation)
    return Candidate(
        mode,
        float(turn1),
        float(turn2),
        float(a),
        float(second),
        length,
        float(wrap_pi(rotation)),
        translation,
        pair.first.point,
        pair.second.point,
        pair.first.tangent,
        pair.second.tangent,
        pair.first.u,
        pair.second.u,
        winding,
    )


def _root_candidates(
    pair: _Pair,
    mode: str,
    a: float,
    second: float,
    settings: Settings,
    *,
    winding_values=range(-2, 3),
    turn_range=(-pi, pi),
) -> tuple[Candidate, ...]:
    delta = wrap_line(pair.second.tangent - pair.first.tangent)
    local_chord = pair.second.point - pair.first.point
    local_angle = atan2(local_chord[1], local_chord[0])

    def residual(turn1: float, winding: int) -> float:
        turn2 = _turn2(delta, winding, turn1, a, second, mode)
        if turn2 is None:
            return np.nan
        x1, y1, x2, y2, tangent1 = _contact_geometry_xy(
            turn1, turn2, a, second, mode
        )
        dx, dy = x2 - x1, y2 - y1
        length = sqrt(dx * dx + dy * dy)
        if length <= 1.0e-12:
            return np.nan
        chord_angle = local_angle + tangent1 - pair.first.tangent
        return (dx * sin(chord_angle) - dy * cos(chord_angle)) / length

    roots: list[tuple[float, int]] = []
    low, high = turn_range
    scan = np.linspace(max(-pi, low), min(pi, high), settings.turn_samples)
    for winding in winding_values:
        values = np.asarray([residual(float(value), winding) for value in scan])
        for i, value in enumerate(values):
            if np.isfinite(value) and abs(value) <= 1.0e-7:
                roots.append((float(scan[i]), int(winding)))
            if i == 0 or not np.isfinite(values[i - 1]) or not np.isfinite(value):
                continue
            if values[i - 1] * value < 0.0:
                try:
                    root = brentq(
                        lambda angle: residual(float(angle), winding),
                        float(scan[i - 1]),
                        float(scan[i]),
                        xtol=1.0e-10,
                        rtol=1.0e-10,
                    )
                    roots.append((float(root), int(winding)))
                except (ValueError, RuntimeError):
                    pass
        for i in range(1, len(scan) - 1):
            if not np.isfinite(values[i - 1 : i + 2]).all():
                continue
            if abs(values[i]) > 1.0e-3 or abs(values[i]) > min(abs(values[i - 1]), abs(values[i + 1])):
                continue
            solved = minimize_scalar(
                lambda angle: abs(residual(float(angle), winding)),
                bounds=(float(scan[i - 1]), float(scan[i + 1])),
                method="bounded",
                options={"xatol": 1.0e-9},
            )
            if solved.success and solved.fun <= 2.0e-6:
                roots.append((float(solved.x), int(winding)))

    candidates = []
    seen = set()
    for root, winding in roots:
        key = (round(root, 7), winding)
        if key in seen:
            continue
        seen.add(key)
        candidate = _construct_candidate(pair, mode, a, second, root, winding)
        if candidate is not None:
            candidates.append(candidate)
    return tuple(candidates)


def _transform(points: Array, candidate: Candidate) -> Array:
    return candidate.translation + rotate(points, candidate.rotation)


def _inverse_transform(points: Array, candidate: Candidate) -> Array:
    return rotate(np.asarray(points) - candidate.translation, -candidate.rotation)


def _capture_for_candidate(candidate: Candidate, samples=41) -> Polygon | None:
    tangent = np.array([0.0, 1.0])
    first_values = np.linspace(candidate.a, 1.0, samples)
    first = arc_point(np.zeros(2), tangent, candidate.turn1, first_values)
    q = first[-1]
    tangent2 = rotate(tangent, candidate.turn1)
    if candidate.mode == "arc2":
        count = max(3, int(np.ceil(samples * candidate.second)))
        second_values = np.linspace(0.0, candidate.second, count)
        second = arc_point(q, tangent2, candidate.turn2, second_values)
        points = np.vstack([first, second[1:]])
    else:
        arc2 = arc_point(q, tangent2, candidate.turn2, np.linspace(0.0, 1.0, samples))
        e2 = arc2[-1]
        terminal = rotate(tangent, candidate.turn1 + candidate.turn2)
        stick = np.linspace(e2, e2 + candidate.second * terminal, max(2, int(samples / 4)))
        points = np.vstack([first, arc2[1:], stick[1:]])
    if len(points) < 3:
        return None
    result = Polygon(candidate.length * points)
    return result if result.is_valid and result.area > 1.0e-9 else None


def _coarse_check(target: Target, candidate: Candidate, settings: Settings):
    extra1 = (candidate.a,)
    extra2 = (candidate.second,) if candidate.mode == "arc2" else ()
    extra_stick = (candidate.second,) if candidate.mode == "stick" else ()
    gripper = build_gripper(
        candidate.turn1,
        candidate.turn2,
        settings.n,
        length=candidate.length,
        samples=41,
        with_region=False,
        extra1=extra1,
        extra2=extra2,
        extra_stick=extra_stick,
    )
    samples = np.vstack([gripper.arc1, gripper.side2])
    local = _inverse_transform(samples, candidate)
    distances = target.distance_field.query(local)
    allowance = max(2.5 * target.distance_field.step, 2.0e-3 * target.major_axis)
    penetration = max(0.0, -float(np.min(distances)) - allowance)

    p1, p2, tangent1, _ = _contact_geometry(
        candidate.turn1,
        candidate.turn2,
        candidate.a,
        candidate.second,
        candidate.mode,
        candidate.length,
    )
    forward = float((p2 - p1) @ np.array([cos(tangent1), sin(tangent1)]))
    capture = _capture_for_candidate(candidate)
    center = Point(*candidate.translation)
    if capture is None:
        capture_gap = 1.0
    elif capture.covers(center):
        capture_gap = 0.0
    else:
        capture_gap = float(capture.distance(center))
    penalty = penetration + capture_gap + max(0.0, -forward)
    passed = penetration <= 0.0 and capture_gap <= 1.0e-9 and forward >= -1.0e-8
    return passed, penalty


def _geometry_coordinates(geometry) -> list[Array]:
    if geometry.is_empty:
        return []
    if isinstance(geometry, Point):
        return [np.asarray(geometry.coords[0], dtype=float)]
    if isinstance(geometry, (LineString,)):
        coordinates = np.asarray(geometry.coords, dtype=float)
        return [coordinates[0], coordinates[-1]]
    if isinstance(geometry, (MultiPoint, MultiLineString, GeometryCollection)):
        return [point for item in geometry.geoms for point in _geometry_coordinates(item)]
    return []


def _contact_position(
    line: LineString,
    boundary,
    *,
    farthest: str,
    low: float,
    high: float,
    tolerance: float,
) -> tuple[float, Array] | None:
    tube = boundary.buffer(tolerance, cap_style=2, join_style=2)
    coordinates = _geometry_coordinates(line.intersection(tube))
    positions = []
    for coordinate in coordinates:
        position = float(line.project(Point(*coordinate)))
        if low - tolerance <= position <= high + tolerance:
            positions.append(float(np.clip(position, low, high)))
    if not positions:
        return None
    position = min(positions) if farthest == "left" else max(positions)
    point = np.asarray(line.interpolate(position).coords[0], dtype=float)
    return position, point


def _target_polygon(target: Target, candidate: Candidate) -> Polygon:
    return orient(Polygon(_transform(target.vertices, candidate)), 1.0)


def evaluate_exact(target: Target, candidate: Candidate, settings: Settings) -> Solution:
    """Apply exact contact, non-penetration, finite-pocket and capture checks."""
    characteristic = max(target.major_axis, 1.0e-9)
    tolerance = settings.exact_tolerance * characteristic
    min_contact_separation = settings.min_contact_separation_relative * characteristic
    min_capture_area = settings.min_capture_area_relative * characteristic**2
    min_centroid_margin = settings.centroid_margin_relative * characteristic
    sample1 = _adaptive_samples(candidate.turn1, tolerance, candidate.length)
    sample2 = _adaptive_samples(candidate.turn2, tolerance, candidate.length)
    gripper = build_gripper(
        candidate.turn1,
        candidate.turn2,
        settings.n,
        length=candidate.length,
        samples=(sample1, sample2),
        with_region=True,
        extra1=(candidate.a,),
        extra2=(candidate.second,) if candidate.mode == "arc2" else (),
        extra_stick=(candidate.second,) if candidate.mode == "stick" else (),
    )
    transformed = _target_polygon(target, candidate)
    violations = []
    if gripper.region is None:
        violations.append("no finite pocket")

    line1, line2 = LineString(gripper.arc1), LineString(gripper.side2)
    inner = transformed.buffer(-tolerance)
    if line1.crosses(transformed) or line2.crosses(transformed):
        violations.append("finger crosses target")
    elif not inner.is_empty and (line1.intersects(inner) or line2.intersects(inner)):
        violations.append("finger enters target interior")

    guard1 = settings.contact_guard * line1.length
    guard2 = settings.contact_guard * LineString(gripper.arc2).length
    first = _contact_position(
        line1,
        transformed.boundary,
        farthest="left",
        low=0.0,
        high=max(0.0, line1.length - guard1),
        tolerance=2.0 * tolerance,
    )
    second = _contact_position(
        line2,
        transformed.boundary,
        farthest="right",
        low=min(line2.length, guard2),
        high=line2.length,
        tolerance=2.0 * tolerance,
    )
    if first is None:
        violations.append("missing P1 contact")
    if second is None:
        violations.append("missing P2 contact")

    p1 = np.full(2, np.nan)
    p2 = np.full(2, np.nan)
    capture = Polygon()
    a_actual = candidate.a
    actual_mode = candidate.mode
    u1, u2 = candidate.u1, candidate.u2
    if first is not None and second is not None:
        distance1, p1 = first
        distance2, p2 = second
        a_actual = float(np.clip(distance1 / line1.length, 0.0, 1.0))
        arc2_length = LineString(gripper.arc2).length
        actual_mode = "arc2" if distance2 <= arc2_length + 2.0 * tolerance else "stick"
        part1 = substring(line1, distance1, line1.length)
        part2 = substring(line2, 0.0, distance2)
        points1 = np.asarray(part1.coords, dtype=float)
        points2 = np.asarray(part2.coords, dtype=float)
        points = np.vstack([points1, points2[1:]])
        capture = Polygon(points)
        if np.linalg.norm(p2 - p1) < min_contact_separation:
            violations.append("contact points too close")
        if not capture.is_valid or capture.area <= min_capture_area:
            violations.append("invalid capture region")
        else:
            centroid = transformed.centroid
            if not capture.contains(centroid):
                violations.append("centroid is outside capture region")
            elif capture.boundary.distance(centroid) < min_centroid_margin:
                violations.append("centroid margin too small")

        tangent_angle = pi / 2.0 + a_actual * candidate.turn1
        tangent = np.array([cos(tangent_angle), sin(tangent_angle)])
        if float((p2 - p1) @ tangent) < -tolerance:
            violations.append("P2 is behind P1")

        target_boundary = transformed.boundary
        u1 = float(target_boundary.project(Point(*p1)) / target_boundary.length)
        u2 = float(target_boundary.project(Point(*p2)) / target_boundary.length)

    feasible = not violations
    if feasible:
        center = np.asarray(transformed.centroid.coords[0], dtype=float)
        # Rank valid grasps by the actual centroid margin of the finite
        # capture region, rather than by a user-tunable proxy cost.
        depth = float(capture.boundary.distance(Point(*center)))
        depth_score = depth / (depth + characteristic)
        p1_score = a_actual
        score = depth_score
    else:
        depth_score = p1_score = score = 0.0
    return Solution(
        gripper,
        transformed,
        actual_mode,
        candidate.length,
        candidate.rotation,
        candidate.translation,
        p1,
        p2,
        a_actual,
        candidate.second,
        u1,
        u2,
        capture,
        float(depth_score),
        float(p1_score),
        float(score),
        feasible,
        tuple(violations),
    )


def _same_solution(left: Solution, right: Solution) -> bool:
    same_turns = (
        left.mode == right.mode
        and abs(left.gripper.turn1 - right.gripper.turn1) < np.radians(2.0)
        and abs(left.gripper.turn2 - right.gripper.turn2) < np.radians(2.0)
    )
    same_contacts = (
        _cyclic_distance(left.u1, right.u1) < 0.01
        and _cyclic_distance(left.u2, right.u2) < 0.01
    )
    same_picture = left.target.hausdorff_distance(right.target) < 1.0e-4
    return same_turns and (same_contacts or same_picture)


def rank_solutions(solutions: Iterable[Solution], count=5) -> tuple[Solution, ...]:
    values = [item for item in solutions if item.feasible]
    if not values:
        return ()
    values.sort(key=lambda item: (item.depth_score, item.p1_score), reverse=True)
    unique = []
    for item in values:
        if any(_same_solution(item, previous) for previous in unique):
            continue
        unique.append(item)
        if len(unique) >= count:
            break
    return tuple(unique)


def _cancelled(cancel) -> bool:
    if cancel is None:
        return False
    if callable(cancel):
        return bool(cancel())
    return bool(cancel.is_set())


def _deadline_decision(
    now: float,
    deadline: float,
    wait_for_first: bool,
    solution_count: int,
    overtime: bool,
) -> tuple[bool, bool]:
    """Return (stop, overtime) for the normal-search/fallback policy."""
    if overtime:
        return solution_count > 0, True
    if now < deadline:
        return False, False
    if wait_for_first and solution_count == 0:
        return False, True
    return True, False


def _candidate_key(candidate: Candidate) -> tuple:
    return (
        candidate.mode,
        round(candidate.turn1, 6),
        round(candidate.turn2, 6),
        round(candidate.a, 5),
        round(candidate.second, 5),
        round(candidate.u1, 5),
        round(candidate.u2, 5),
    )


def solve_target(
    target: Target,
    settings: Settings = Settings(),
    progress: Progress | None = None,
    cancel=None,
) -> tuple[Solution, ...]:
    """Fit a fixed-size target by choosing arc length and rigid pose."""
    settings = settings.checked()
    started = time.perf_counter()
    deadline = started + settings.time_limit
    all_solutions: list[Solution] = []
    promising: list[tuple[float, Candidate, _Pair]] = []
    tested: set[tuple] = set()
    failure_counts: dict[str, int] = {}
    pairs_done = exact_checks = roots_found = 0
    feature_samples = 0
    levels_processed = 0
    overtime = False

    def notify(stage: str, **extra):
        if progress:
            progress(
                {
                    "stage": stage,
                    "elapsed": time.perf_counter() - started,
                    "pairs": pairs_done,
                    "roots": roots_found,
                    "exact": exact_checks,
                    "solutions": len(all_solutions),
                    "feature_samples": feature_samples,
                    "levels": levels_processed,
                    "overtime": overtime,
                    **extra,
                }
            )

    def stop_requested() -> bool:
        nonlocal overtime
        if _cancelled(cancel):
            return True
        stop, next_overtime = _deadline_decision(
            time.perf_counter(),
            deadline,
            settings.wait_for_first,
            len(all_solutions),
            overtime,
        )
        if next_overtime and not overtime:
            overtime = next_overtime
            notify("time limit exceeded; waiting for first solution")
        return stop

    def inspect(candidate: Candidate, pair: _Pair):
        nonlocal exact_checks, roots_found
        if stop_requested():
            return
        key = _candidate_key(candidate)
        if key in tested:
            return
        tested.add(key)
        roots_found += 1
        passed, penalty = _coarse_check(target, candidate, settings)
        promising.append((penalty, candidate, pair))
        if not passed:
            return
        exact_checks += 1
        solution = evaluate_exact(target, candidate, settings)
        if solution.feasible:
            all_solutions.append(solution)
        else:
            for reason in solution.violations:
                failure_counts[reason] = failure_counts.get(reason, 0) + 1

    for stage, level in enumerate(settings.boundary_levels):
        if stop_requested():
            break
        cone_step = (45.0, 15.0, 5.0)[min(stage, 2)]
        features = boundary_features(target, level, cone_step=cone_step)
        feature_samples += len(features)
        levels_processed += 1
        if stop_requested():
            break
        pairs = _contact_pairs(features, settings, target, should_stop=stop_requested)
        if stop_requested():
            break
        limit = settings.pair_limits[min(stage, len(settings.pair_limits) - 1)]
        pairs = _select_pairs(
            pairs,
            min(limit, len(pairs)),
            should_stop=stop_requested,
        )
        if stop_requested():
            break
        notify(
            "contact pairs",
            level=level,
            pair_total=len(pairs),
            feature_samples=feature_samples,
            levels=levels_processed,
        )

        a_values = np.linspace(0.0, 1.0 - settings.contact_guard, settings.contact_grid)
        arc_values = np.linspace(settings.contact_guard, 1.0, settings.contact_grid)
        stick_values = np.linspace(0.0, settings.n, settings.contact_grid) if settings.n > 0 else ()
        for pair in pairs:
            if stop_requested():
                break
            pairs_done += 1
            for mode, second_values in (("arc2", arc_values), ("stick", stick_values)):
                for a in a_values:
                    for second in second_values:
                        if stop_requested():
                            break
                        for candidate in _root_candidates(
                            pair, mode, float(a), float(second), settings
                        ):
                            inspect(candidate, pair)
                    if stop_requested():
                        break
            if pairs_done % 10 == 0:
                notify("constructing contacts", level=level, pair_total=len(pairs))

        if stop_requested():
            break

        promising.sort(key=lambda item: item[0])
        seeds = promising[:64]
        base_a = (1.0 - settings.contact_guard) / (settings.contact_grid - 1)
        base_second = 1.0 / (settings.contact_grid - 1)
        for refinement in range(settings.refinement_levels):
            if stop_requested():
                break
            notify("refining", level=level, refinement=refinement + 1)
            da = base_a / (2.0 ** (refinement + 1))
            for _, seed, pair in seeds:
                second_span = settings.n if seed.mode == "stick" else 1.0 - settings.contact_guard
                ds = base_second * second_span / (2.0 ** (refinement + 1))
                a_options = np.clip([seed.a - da, seed.a, seed.a + da], 0.0, 1.0 - settings.contact_guard)
                if seed.mode == "arc2":
                    second_options = np.clip(
                        [seed.second - ds, seed.second, seed.second + ds],
                        settings.contact_guard,
                        1.0,
                    )
                else:
                    second_options = np.clip(
                        [seed.second - ds, seed.second, seed.second + ds], 0.0, settings.n
                    )
                half_width = 2.0 * pi / max(settings.turn_samples - 1, 1)
                turn_range = (seed.turn1 - half_width, seed.turn1 + half_width)
                for a in np.unique(a_options):
                    for second in np.unique(second_options):
                        for candidate in _root_candidates(
                            pair,
                            seed.mode,
                            float(a),
                            float(second),
                            settings,
                            winding_values=(seed.winding,),
                            turn_range=turn_range,
                        ):
                            inspect(candidate, pair)
                        if stop_requested():
                            break
                    if stop_requested():
                        break
                if stop_requested():
                    break
            promising.sort(key=lambda item: item[0])
            seeds = promising[:64]

        rank_count = 1 if overtime else settings.solution_count
        ranked = rank_solutions(all_solutions, rank_count)
        if len(ranked) >= rank_count:
            recent = time.perf_counter() - started
            if recent >= min(settings.time_limit, 1.5 + 0.55 * settings.time_limit):
                break

    rank_count = 1 if overtime else settings.solution_count
    ranked = rank_solutions(all_solutions, rank_count)
    if _cancelled(cancel):
        notify("cancelled", result_count=len(ranked))
        return ranked
    if not ranked:
        details = ", ".join(
            f"{name}: {value}" for name, value in sorted(failure_counts.items(), key=lambda x: -x[1])[:3]
        )
        suffix = f" ({details})" if details else ""
        notify("no solution", result_count=0)
        scope = "after the complete overtime search" if overtime else f"within {settings.time_limit:.1f}s"
        hint = "" if overtime else "; increase --time-limit or use --wait-first"
        raise DesignError(f"no exact forward solution found {scope}{suffix}{hint}")
    notify(
        "finished",
        result_count=len(ranked),
        min_length=min(item.length for item in ranked),
        max_length=max(item.length for item in ranked),
        feature_samples=feature_samples,
        levels=levels_processed,
        failure_counts=dict(failure_counts),
    )
    return ranked
