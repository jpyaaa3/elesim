"""Coarse collision-proxy checks for motion planning.

Each arm link is approximated as a capsule (a line segment plus radius)
fixed in that link's own FK frame -- generated offline from that link's
mesh, see
``misc/tooling/model_builder/src/elesim_model_builder/collision_model.py``.
A capsule rather than a single bounding sphere: several base-assembly parts
are long and thin, and a single sphere centered anywhere on them has to
cover their *entire* length. A capsule's radius only has to cover the
part's *cross-section*, so it stays tight for elongated parts while still
degenerating to (approximately) a sphere for compact ones.

Two base-assembly parts (``plate``, ``housing``) are flat/wide rather than
elongated -- a capsule's single radius has to cover their *widest*
cross-section, which (measured against the real mesh geometry) still
overlapped a large, pose-independent fraction of the arm's own reachable
range even with a correctly-fit capsule. Those two are instead approximated
as an oriented box (``LinkBox``), whose three independent half-extents fit
a flat part's actual thin cross-section; see ``capsule_box_gap``.

GO2's chassis is approximated the same way, as a small number of capsules
in the body-local frame. This all trades geometric precision for simple,
well-tested distance math suited to a planning-time coarse check; it is
not a substitute for the Simulator's physics collision geometry.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, replace
from typing import Any, Mapping, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from elesim_controller.robot.arm.iklib import kinematics as ik_kin

Vec3 = np.ndarray


def closest_point_on_segment(p: Sequence[float], a: Sequence[float], b: Sequence[float]) -> np.ndarray:
    p_arr = np.asarray(p, dtype=float).reshape(3)
    a_arr = np.asarray(a, dtype=float).reshape(3)
    b_arr = np.asarray(b, dtype=float).reshape(3)
    ab = b_arr - a_arr
    denom = float(np.dot(ab, ab))
    if denom <= 1e-12:
        return a_arr.copy()
    t = float(np.clip(np.dot(p_arr - a_arr, ab) / denom, 0.0, 1.0))
    return a_arr + t * ab


def point_segment_distance(p: Sequence[float], a: Sequence[float], b: Sequence[float]) -> float:
    closest = closest_point_on_segment(p, a, b)
    return float(np.linalg.norm(np.asarray(p, dtype=float).reshape(3) - closest))


def segment_segment_distance(
    a0: Sequence[float], a1: Sequence[float], b0: Sequence[float], b1: Sequence[float]
) -> float:
    """Closest distance between two 3D line segments (Ericson's ClosestPtSegmentSegment).

    Plain Python floats, not numpy -- same rationale as ``segment_box_distance``
    and ``box_box_gap``: called tens of thousands of times per plan/shortcut/
    smoothing pass (self-collision's capsule-vs-capsule checks) on tiny
    (3-element) vectors, where numpy's per-call overhead swamps the actual
    arithmetic.
    """
    ax0, ay0, az0 = (float(x) for x in a0)
    ax1, ay1, az1 = (float(x) for x in a1)
    bx0, by0, bz0 = (float(x) for x in b0)
    bx1, by1, bz1 = (float(x) for x in b1)

    d1x, d1y, d1z = ax1 - ax0, ay1 - ay0, az1 - az0
    d2x, d2y, d2z = bx1 - bx0, by1 - by0, bz1 - bz0
    rx, ry, rz = ax0 - bx0, ay0 - by0, az0 - bz0

    a = d1x * d1x + d1y * d1y + d1z * d1z
    e = d2x * d2x + d2y * d2y + d2z * d2z
    f = d2x * rx + d2y * ry + d2z * rz
    eps = 1e-12

    if a <= eps and e <= eps:
        return math.sqrt(rx * rx + ry * ry + rz * rz)
    if a <= eps:
        s = 0.0
        t = min(max(f / e, 0.0), 1.0)
    else:
        c = d1x * rx + d1y * ry + d1z * rz
        if e <= eps:
            t = 0.0
            s = min(max(-c / a, 0.0), 1.0)
        else:
            b = d1x * d2x + d1y * d2y + d1z * d2z
            denom = a * e - b * b
            s = min(max((b * f - c * e) / denom, 0.0), 1.0) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = min(max(-c / a, 0.0), 1.0)
            elif t > 1.0:
                t = 1.0
                s = min(max((b - c) / a, 0.0), 1.0)

    cax, cay, caz = ax0 + d1x * s, ay0 + d1y * s, az0 + d1z * s
    cbx, cby, cbz = bx0 + d2x * t, by0 + d2y * t, bz0 + d2z * t
    dx, dy, dz = cax - cbx, cay - cby, caz - cbz
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def capsule_capsule_gap(
    a0: Sequence[float], a1: Sequence[float], ra: float, b0: Sequence[float], b1: Sequence[float], rb: float
) -> float:
    """Surface-to-surface gap between two capsules; negative means overlap."""
    return segment_segment_distance(a0, a1, b0, b1) - float(ra) - float(rb)


# See segment_box_distance's docstring for the precision-vs-cost tradeoff.
_GOLDEN_SECTION_ITERS = 20


def segment_box_distance(
    a_world: Sequence[float],
    b_world: Sequence[float],
    box_center: Sequence[float],
    half_extents: Sequence[float],
    box_rot: np.ndarray,
) -> float:
    """Closest (unsigned) distance from a line segment to an oriented box; 0 if the segment enters it.

    ``t -> distance(a + t*(b-a), box)`` is a convex function of ``t``:
    distance-to-a-convex-set is itself convex, and composing a convex
    function with the affine map ``t -> a + t*(b-a)`` preserves convexity.
    That guarantees golden-section search over ``t in [0, 1]`` converges to
    the true global minimum -- exact up to floating-point precision, not a
    sampling-based approximation -- in a small, fixed number of iterations.

    ``_GOLDEN_SECTION_ITERS`` iterations shrink the bracket by
    ``((sqrt(5)-1)/2)**iters``; at 20 that's ~1e-4, i.e. sub-0.1mm precision
    even for a full 1m segment -- far finer than any clearance tolerance
    used elsewhere (cm-scale). Profiled live: this was the single largest
    cost in the whole plan/shortcut/smoothing pipeline (~40 calls to `f`
    per invocation, called ~125k times threading a wall opening -- over
    5 million total, ~50s of an ~80s run) purely from an iteration count
    (40) far beyond what the precision requirement needed.

    The search itself (below) works in plain Python floats, not numpy,
    despite the setup above using numpy for the one-time frame change --
    profiled live: with the iteration count already fixed, numpy's fixed
    per-call overhead on tiny 3-element arrays (norm/maximum/abs, each
    dispatching through numpy's generic machinery) dominated the *actual*
    arithmetic when done ~20-40 times per invocation across hundreds of
    thousands of invocations. Unpacking to scalars once and using
    math.sqrt/builtin abs/max in the hot loop is the same computation,
    just without that per-call tax.
    """
    rot = np.asarray(box_rot, dtype=float).reshape(3, 3)
    half = np.asarray(half_extents, dtype=float).reshape(3)
    center = np.asarray(box_center, dtype=float).reshape(3)
    a_local = rot.T @ (np.asarray(a_world, dtype=float).reshape(3) - center)
    b_local = rot.T @ (np.asarray(b_world, dtype=float).reshape(3) - center)
    direction = b_local - a_local

    ax, ay, az = float(a_local[0]), float(a_local[1]), float(a_local[2])
    dx, dy, dz = float(direction[0]), float(direction[1]), float(direction[2])
    hx, hy, hz = float(half[0]), float(half[1]), float(half[2])

    def f(t: float) -> float:
        ox = abs(ax + t * dx) - hx
        oy = abs(ay + t * dy) - hy
        oz = abs(az + t * dz) - hz
        if ox < 0.0:
            ox = 0.0
        if oy < 0.0:
            oy = 0.0
        if oz < 0.0:
            oz = 0.0
        return math.sqrt(ox * ox + oy * oy + oz * oz)

    golden = (math.sqrt(5.0) - 1.0) / 2.0
    lo, hi = 0.0, 1.0
    c = hi - golden * (hi - lo)
    d = lo + golden * (hi - lo)
    fc, fd = f(c), f(d)
    for _ in range(_GOLDEN_SECTION_ITERS):
        if fc < fd:
            hi, d, fd = d, c, fc
            c = hi - golden * (hi - lo)
            fc = f(c)
        else:
            lo, c, fc = c, d, fd
            d = lo + golden * (hi - lo)
            fd = f(d)
    return f((lo + hi) / 2.0)


def capsule_box_gap(
    a_world: Sequence[float],
    b_world: Sequence[float],
    radius: float,
    box_center: Sequence[float],
    half_extents: Sequence[float],
    box_rot: np.ndarray,
) -> float:
    """Surface-to-surface gap between a capsule and an oriented box; negative means overlap."""
    return segment_box_distance(a_world, b_world, box_center, half_extents, box_rot) - float(radius)


def box_box_gap(
    a_center: Sequence[float],
    a_half_extents: Sequence[float],
    a_rot: np.ndarray,
    b_center: Sequence[float],
    b_half_extents: Sequence[float],
    b_rot: np.ndarray,
) -> float:
    """Surface-to-surface gap between two oriented boxes via the separating axis theorem.

    An earlier corner-sampling approach (check each box's 8 corners against
    the other box's surface) missed a common real case: two boxes that
    overlap face-to-face along one axis while perfectly aligned on the
    other two never have a corner *inside* the other box, even though a
    real overlap volume exists between their faces -- confirmed by a
    direct check (two unit-ish boxes offset only along X reported gap=0.0,
    "just touching", for a configuration that actually overlaps by 0.05m).

    SAT is the standard, exact way to test two convex polytopes: 15
    candidate separating axes (each box's own 3 face normals, plus all 9
    pairwise cross products of their edge directions) are checked; two
    boxes are disjoint iff *some* axis separates their projections. If one
    does, the largest per-axis gap is a valid (though not always exact)
    lower bound on the true 3D distance -- conservative in the safe
    direction, since it can only *under*-report clearance, never mask a
    real collision by over-reporting it. If no axis separates them, they
    truly overlap, and the least-negative per-axis overlap is the standard
    "minimum translation distance" proxy for how deep the intersection is.

    All 15 axes are evaluated in plain Python floats, not numpy -- profiled
    live twice now: first a per-axis Python loop of individual numpy calls
    was the 2nd-largest cost in the plan/shortcut/smoothing pipeline, fixed
    by batching all 15 axes into one set of numpy array ops; re-profiled
    later and *that* batched-numpy version was still a top-3 cost, because
    numpy's fixed per-call overhead on such tiny (3-15 element) arrays
    swamps the actual arithmetic even in one batched call. Same fix as
    ``segment_box_distance``'s golden-section loop: unpack to scalars once,
    do the 15-axis SAT check with plain dot/cross products and builtin
    abs/max. Degenerate (near-parallel edge) cross-product axes are simply
    skipped, same as the original's ``if norm > 1e-9`` guard.
    """
    a_rot_arr = np.asarray(a_rot, dtype=float).reshape(3, 3)
    b_rot_arr = np.asarray(b_rot, dtype=float).reshape(3, 3)
    a_half = tuple(float(x) for x in np.asarray(a_half_extents, dtype=float).reshape(3))
    b_half = tuple(float(x) for x in np.asarray(b_half_extents, dtype=float).reshape(3))
    ac = np.asarray(a_center, dtype=float).reshape(3)
    bc = np.asarray(b_center, dtype=float).reshape(3)
    dx, dy, dz = float(bc[0] - ac[0]), float(bc[1] - ac[1]), float(bc[2] - ac[2])

    # Each box's local axes as world-frame 3-tuples (columns of its rotation matrix).
    a_axes = [(float(a_rot_arr[0, i]), float(a_rot_arr[1, i]), float(a_rot_arr[2, i])) for i in range(3)]
    b_axes = [(float(b_rot_arr[0, j]), float(b_rot_arr[1, j]), float(b_rot_arr[2, j])) for j in range(3)]
    ax0, ay0, az0 = a_axes[0]
    ax1, ay1, az1 = a_axes[1]
    ax2, ay2, az2 = a_axes[2]
    bx0, by0, bz0 = b_axes[0]
    bx1, by1, bz1 = b_axes[1]
    bx2, by2, bz2 = b_axes[2]
    ah0, ah1, ah2 = a_half
    bh0, bh1, bh2 = b_half

    # Fully unrolled (no per-axis _dot()/sum() call, no generator) -- this
    # runs ~15 times per box_box_gap call, called tens of thousands of
    # times per plan/shortcut/smoothing pass; profiled live as still a
    # meaningful chunk of time even after switching to plain floats, from
    # the function-call/generator overhead alone (2.3M _dot() calls in one
    # run). Same arithmetic, just inlined.
    def _gap_for_axis(ux: float, uy: float, uz: float) -> float:
        radius_a = (
            ah0 * abs(ux * ax0 + uy * ay0 + uz * az0)
            + ah1 * abs(ux * ax1 + uy * ay1 + uz * az1)
            + ah2 * abs(ux * ax2 + uy * ay2 + uz * az2)
        )
        radius_b = (
            bh0 * abs(ux * bx0 + uy * by0 + uz * bz0)
            + bh1 * abs(ux * bx1 + uy * by1 + uz * bz1)
            + bh2 * abs(ux * bx2 + uy * by2 + uz * bz2)
        )
        center_distance = abs(dx * ux + dy * uy + dz * uz)
        return center_distance - radius_a - radius_b

    worst = -math.inf
    for axis in a_axes:
        worst = max(worst, _gap_for_axis(*axis))
    for axis in b_axes:
        worst = max(worst, _gap_for_axis(*axis))
    for au in a_axes:
        for bu in b_axes:
            cx = au[1] * bu[2] - au[2] * bu[1]
            cy = au[2] * bu[0] - au[0] * bu[2]
            cz = au[0] * bu[1] - au[1] * bu[0]
            norm = math.sqrt(cx * cx + cy * cy + cz * cz)
            if norm <= 1e-9:
                continue
            worst = max(worst, _gap_for_axis(cx / norm, cy / norm, cz / norm))
    return worst


@dataclass(frozen=True)
class Go2BodyCapsule:
    p0_body: tuple[float, float, float]
    p1_body: tuple[float, float, float]
    radius: float
    label: str = ""


@dataclass(frozen=True)
class Go2BodyBox:
    """A GO2-body-rigid box proxy (e.g. the torso), in GO2's own base-link frame."""

    center_body: tuple[float, float, float]
    half_extents_body: tuple[float, float, float]
    rot_body: tuple[
        tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]
    ] = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    label: str = ""


@dataclass(frozen=True)
class WorldBox:
    """A static obstacle box, already in world coordinates (not attached to
    GO2 or the arm -- e.g. a wall, a shelf, anything fixed in the scene)."""

    center_world: tuple[float, float, float]
    half_extents_world: tuple[float, float, float]
    rot_world: tuple[
        tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]
    ] = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    label: str = ""


@dataclass(frozen=True)
class WorldCapsule:
    """A static obstacle capsule, already in world coordinates (not attached
    to GO2 or the arm) -- e.g. a cylindrical post/pillar, modeled as a line
    segment plus radius the same way arm links are (see this module's
    docstring): no new distance math needed, since ``capsule_capsule_gap``/
    ``capsule_box_gap`` already handle a capsule vs. any arm-link shape."""

    p0_world: tuple[float, float, float]
    p1_world: tuple[float, float, float]
    radius: float
    label: str = ""


@dataclass(frozen=True)
class Go2LegSegment:
    """One leg segment (hip/thigh/calf)'s static geometry, for ``go2_leg_world_shapes``.

    Unlike ``Go2BodyCapsule``/``Go2BodyBox`` (rigidly fixed to GO2's base,
    e.g. the torso and head), each leg segment moves as GO2 walks -- its
    joint (``origin_from_parent``/``axis_in_parent``, both already in the
    *parent* segment's frame, matching the arm's own ``fk_joint_chain``
    convention) rotates by a live angle, ``leg_q[leg_q_index]``. ``parent``
    is either ``"base"`` (for a hip) or another leg segment's ``name`` (for
    a thigh/calf) -- always already computed by the time a child needs it,
    since segments are listed hip-before-thigh-before-calf per leg.
    """

    name: str
    parent: str
    origin_from_parent: tuple[float, float, float]
    origin_rot_in_parent: tuple[
        tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]
    ]
    axis_in_parent: tuple[float, float, float]
    leg_q_index: int
    shape_type: str  # "capsule" | "box"
    p0_local: tuple[float, float, float] = (0.0, 0.0, 0.0)
    p1_local: tuple[float, float, float] = (0.0, 0.0, 0.0)
    radius: float = 0.0
    center_local: tuple[float, float, float] = (0.0, 0.0, 0.0)
    half_extents_local: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rot_local: tuple[
        tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]
    ] = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


@dataclass(frozen=True)
class LinkCapsule:
    """One link's collision proxy, in that link's own local FK frame."""

    p0_local: tuple[float, float, float]
    p1_local: tuple[float, float, float]
    radius: float


@dataclass(frozen=True)
class LinkBox:
    """One link's collision proxy as an oriented box, in that link's own local FK frame.

    For flat/wide parts (``plate``, ``housing``) a capsule's single radius
    has to cover the part's *widest* cross-section, which -- measured
    against the real mesh geometry -- overlapped a large fraction of the
    arm's own reachable pose space even after fixing the capsule-fitting
    bug that produced a wildly oversized sphere (see ``chain_axis_capsule``
    in the model-builder). A box's three independent half-extents fit a
    flat part's actual thin cross-section instead of inflating it to match
    the widest axis.
    """

    center_local: tuple[float, float, float]
    half_extents_local: tuple[float, float, float]


@dataclass(frozen=True)
class CollisionModel:
    """Static collision-proxy geometry for one arm model.

    ``link_capsules`` keys must match the ``fk_joint_chain`` link names in
    ``arm_model.json`` (``plate``, ``housing``, ``wedge``, ``node0``..``node9``,
    ``gripper_base``, ``gripper_claw_left``, ``gripper_claw_right``, ``camera``).
    A link with no entry in either ``link_capsules`` or ``link_boxes`` falls
    back to a degenerate (point) capsule of ``default_radius`` at its own
    frame origin. A link name should appear in at most one of
    ``link_capsules``/``link_boxes`` -- when both sides of a pair are boxes
    (e.g. ``plate`` vs ``gripper_base``), ``box_box_gap`` handles it (an
    approximate corner-sampling check, see its docstring for the accepted
    limitation).

    ``link_boxes`` maps a link name to *one or more* boxes -- some parts
    (``plate``) have two sub-features far apart along one axis (a thin main
    body plus a compute module mounted at its far end), and a single
    axis-aligned box has to cover the full span between them, inflating a
    "no part actually here" gap to the taller sub-feature's height. Fitting
    each sub-feature its own box instead keeps both tight; a bare list of
    two disjoint boxes correctly reports "clear" for anything that only
    enters that gap.
    """

    link_capsules: Mapping[str, LinkCapsule]
    link_boxes: Mapping[str, tuple[LinkBox, ...]] = field(default_factory=dict)
    default_radius: float = 0.03
    go2_capsules: tuple[Go2BodyCapsule, ...] = ()
    go2_boxes: tuple[Go2BodyBox, ...] = ()
    go2_leg_segments: tuple[Go2LegSegment, ...] = ()
    # Static scene obstacles (walls, shelves, ...), already in world
    # coordinates -- unlike ``go2_boxes``/``go2_capsules`` these are not
    # attached to GO2's base and don't move with it. See
    # ``environment_collision_check``.
    obstacle_boxes: tuple[WorldBox, ...] = ()
    # Cylindrical/round static obstacles (posts, pillars, ...), modeled as
    # capsules -- see ``WorldCapsule``.
    obstacle_capsules: tuple[WorldCapsule, ...] = ()
    ignore_pairs: frozenset[frozenset[str]] = field(default_factory=frozenset)
    # `plate` is bolted directly onto GO2's back and `housing` sits on
    # `plate` -- both are *meant* to sit at/overlap the chassis surface by
    # construction (confirmed live: the arm's ordinary resting pose reports
    # `plate` vs the GO2 chassis capsule at -0.037m, i.e. always in
    # "collision" with the body it's mounted to). That's a mounting, not a
    # hazard, so both stay excluded from the GO2-body check by default.
    go2_ignore_links: frozenset[str] = frozenset({"plate", "housing"})
    # Self-collision (vs the arm's *other* links) is a different question:
    # `plate`/`housing` used to default to wholesale-ignored here too, but
    # that was masking a real bug -- their capsule fit collapsed to a giant
    # single-point sphere (0.555m radius for `plate`, confirmed live -- see
    # ``elesim_model_builder.collision_model.chain_axis_capsule``), which
    # falsely overlapped nearly every pose. Now that the model-builder fits
    # them properly (as boxes, see ``LinkBox``), they're checked like any
    # other link by default; any pair that always-overlaps by construction
    # is caught by ``discover_always_colliding_pairs`` and folded into
    # ``ignore_pairs`` instead of being excluded wholesale.
    self_ignore_links: frozenset[str] = frozenset()

    def capsule_for(self, link_name: str) -> LinkCapsule:
        capsule = self.link_capsules.get(str(link_name))
        if capsule is not None:
            return capsule
        return LinkCapsule(p0_local=(0.0, 0.0, 0.0), p1_local=(0.0, 0.0, 0.0), radius=self.default_radius)

    def world_capsule(self, link_name: str, pos: np.ndarray, rot: np.ndarray) -> tuple[np.ndarray, np.ndarray, float]:
        """Map this link's local capsule into world coordinates."""
        capsule = self.capsule_for(link_name)
        pos_arr = np.asarray(pos, dtype=float).reshape(3)
        rot_arr = np.asarray(rot, dtype=float).reshape(3, 3)
        p0_world = pos_arr + rot_arr @ np.asarray(capsule.p0_local, dtype=float)
        p1_world = pos_arr + rot_arr @ np.asarray(capsule.p1_local, dtype=float)
        return p0_world, p1_world, capsule.radius

    def is_box(self, link_name: str) -> bool:
        return str(link_name) in self.link_boxes

    def world_boxes(
        self, link_name: str, pos: np.ndarray, rot: np.ndarray
    ) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
        """Map this link's local box(es) into world coordinates: [(center, half_extents, rotation), ...]."""
        pos_arr = np.asarray(pos, dtype=float).reshape(3)
        rot_arr = np.asarray(rot, dtype=float).reshape(3, 3)
        return [
            (pos_arr + rot_arr @ np.asarray(box.center_local, dtype=float), np.asarray(box.half_extents_local, dtype=float), rot_arr)
            for box in self.link_boxes[str(link_name)]
        ]

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CollisionModel":
        link_capsules = {
            str(name): LinkCapsule(
                p0_local=tuple(float(x) for x in entry["p0_local"]),
                p1_local=tuple(float(x) for x in entry["p1_local"]),
                radius=float(entry["radius"]),
            )
            for name, entry in dict(data.get("link_capsules", {}) or {}).items()
        }
        link_boxes = {
            str(name): tuple(
                LinkBox(
                    center_local=tuple(float(x) for x in box["center_local"]),
                    half_extents_local=tuple(float(x) for x in box["half_extents_local"]),
                )
                for box in entries
            )
            for name, entries in dict(data.get("link_boxes", {}) or {}).items()
        }
        go2_capsules = tuple(
            Go2BodyCapsule(
                p0_body=tuple(float(x) for x in entry["p0_body"]),
                p1_body=tuple(float(x) for x in entry["p1_body"]),
                radius=float(entry["radius"]),
                label=str(entry.get("label", "")),
            )
            for entry in data.get("go2_capsules", []) or []
        )
        go2_boxes = tuple(
            Go2BodyBox(
                center_body=tuple(float(x) for x in entry["center_body"]),
                half_extents_body=tuple(float(x) for x in entry["half_extents_body"]),
                rot_body=tuple(tuple(float(x) for x in row) for row in entry["rot_body"]),
                label=str(entry.get("label", "")),
            )
            for entry in data.get("go2_boxes", []) or []
        )
        go2_leg_segments = tuple(
            Go2LegSegment(
                name=str(entry["name"]),
                parent=str(entry["parent"]),
                origin_from_parent=tuple(float(x) for x in entry["origin_from_parent"]),
                origin_rot_in_parent=tuple(tuple(float(x) for x in row) for row in entry["origin_rot_in_parent"]),
                axis_in_parent=tuple(float(x) for x in entry["axis_in_parent"]),
                leg_q_index=int(entry["leg_q_index"]),
                shape_type=str(entry["shape_type"]),
                p0_local=tuple(float(x) for x in entry["p0_local"]) if "p0_local" in entry else (0.0, 0.0, 0.0),
                p1_local=tuple(float(x) for x in entry["p1_local"]) if "p1_local" in entry else (0.0, 0.0, 0.0),
                radius=float(entry.get("radius", 0.0)),
                center_local=tuple(float(x) for x in entry["center_local"]) if "center_local" in entry else (0.0, 0.0, 0.0),
                half_extents_local=(
                    tuple(float(x) for x in entry["half_extents_local"]) if "half_extents_local" in entry else (0.0, 0.0, 0.0)
                ),
                rot_local=(
                    tuple(tuple(float(x) for x in row) for row in entry["rot_local"])
                    if "rot_local" in entry
                    else ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
                ),
            )
            for entry in data.get("go2_leg_segments", []) or []
        )
        obstacle_boxes = tuple(
            WorldBox(
                center_world=tuple(float(x) for x in entry["center_world"]),
                half_extents_world=tuple(float(x) for x in entry["half_extents_world"]),
                rot_world=(
                    tuple(tuple(float(x) for x in row) for row in entry["rot_world"])
                    if "rot_world" in entry
                    else ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
                ),
                label=str(entry.get("label", "")),
            )
            for entry in data.get("obstacle_boxes", []) or []
        )
        obstacle_capsules = tuple(
            WorldCapsule(
                p0_world=tuple(float(x) for x in entry["p0_world"]),
                p1_world=tuple(float(x) for x in entry["p1_world"]),
                radius=float(entry["radius"]),
                label=str(entry.get("label", "")),
            )
            for entry in data.get("obstacle_capsules", []) or []
        )
        ignore_pairs = frozenset(
            frozenset((str(pair[0]), str(pair[1]))) for pair in data.get("ignore_pairs", []) or []
        )
        go2_ignore_raw = data.get("go2_ignore_links")
        go2_ignore = (
            frozenset({"plate", "housing"})
            if go2_ignore_raw is None
            else frozenset(str(x) for x in go2_ignore_raw)
        )
        self_ignore_raw = data.get("self_ignore_links")
        self_ignore = frozenset() if self_ignore_raw is None else frozenset(str(x) for x in self_ignore_raw)
        return cls(
            link_capsules=link_capsules,
            link_boxes=link_boxes,
            default_radius=float(data.get("default_radius", 0.03)),
            go2_capsules=go2_capsules,
            go2_boxes=go2_boxes,
            go2_leg_segments=go2_leg_segments,
            obstacle_boxes=obstacle_boxes,
            obstacle_capsules=obstacle_capsules,
            ignore_pairs=ignore_pairs,
            go2_ignore_links=go2_ignore,
            self_ignore_links=self_ignore,
        )

    @classmethod
    def from_json(cls, path: str) -> "CollisionModel":
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        if int(data.get("schema_version", 0)) != 1:
            raise RuntimeError(f"unsupported collision model schema: {path}")
        return cls.from_dict(data)


def load_collision_model_for_config(config_path: str) -> Optional[CollisionModel]:
    """Best-effort load of the sibling collision-model artifact for an IK config.

    Mirrors ``load_solver_context``'s resolution of ``arm_model.json``: an
    ``ELESIM_COLLISION_MODEL`` env override, else ``collision_model.json``
    next to ``config_path``. Unlike the arm model, this artifact is
    optional -- planned-move collision checking is simply unavailable
    (returns ``None``) if it hasn't been generated yet.
    """
    model_path = os.environ.get("ELESIM_COLLISION_MODEL", "").strip()
    if not model_path:
        model_path = os.path.join(os.path.dirname(os.path.abspath(config_path)), "collision_model.json")
    if not os.path.isfile(model_path):
        return None
    return CollisionModel.from_json(model_path)


@dataclass(frozen=True)
class CollisionResult:
    ok: bool
    min_clearance_m: float
    link_a: str
    link_b: str
    reason: str = ""


def adjacent_link_pairs(fk_joint_chain: Sequence[Mapping[str, Any]]) -> frozenset[frozenset[str]]:
    """Pairs that are mechanically close by construction -- always excluded.

    Two links sharing a joint are rigidly connected at that joint's anchor,
    so their capsules always touch there. Two links that are both direct
    children of the same parent (e.g. the two gripper claws, or a claw and
    the wrist camera) are likewise expected to sit close together at the
    parent's anchor; neither case is a self-collision.
    """
    pairs: set[frozenset[str]] = set()
    children_by_parent: dict[str, list[str]] = {}
    for meta in fk_joint_chain:
        parent = str(meta["parent"])
        child = str(meta["child"])
        pairs.add(frozenset((parent, child)))
        children_by_parent.setdefault(parent, []).append(child)
    for children in children_by_parent.values():
        for i in range(len(children)):
            for j in range(i + 1, len(children)):
                pairs.add(frozenset((children[i], children[j])))
    return frozenset(pairs)


def _link_pair_gap(
    model: CollisionModel,
    name_a: str,
    pos_a: np.ndarray,
    rot_a: np.ndarray,
    name_b: str,
    pos_b: np.ndarray,
    rot_b: np.ndarray,
) -> float:
    """Surface-to-surface gap between two links, dispatching capsule/box/box-box per link."""
    a_is_box = model.is_box(name_a)
    b_is_box = model.is_box(name_b)
    if a_is_box and b_is_box:
        return min(
            box_box_gap(a_center, a_half, a_rot, b_center, b_half, b_rot)
            for a_center, a_half, a_rot in model.world_boxes(name_a, pos_a, rot_a)
            for b_center, b_half, b_rot in model.world_boxes(name_b, pos_b, rot_b)
        )
    if a_is_box:
        b0, b1, rb = model.world_capsule(name_b, pos_b, rot_b)
        return min(
            capsule_box_gap(b0, b1, rb, box_center, half_extents, box_rot)
            for box_center, half_extents, box_rot in model.world_boxes(name_a, pos_a, rot_a)
        )
    if b_is_box:
        a0, a1, ra = model.world_capsule(name_a, pos_a, rot_a)
        return min(
            capsule_box_gap(a0, a1, ra, box_center, half_extents, box_rot)
            for box_center, half_extents, box_rot in model.world_boxes(name_b, pos_b, rot_b)
        )
    a0, a1, ra = model.world_capsule(name_a, pos_a, rot_a)
    b0, b1, rb = model.world_capsule(name_b, pos_b, rot_b)
    return capsule_capsule_gap(a0, a1, ra, b0, b1, rb)


def self_collision_check(
    link_tf: Mapping[str, tuple[np.ndarray, np.ndarray]],
    *,
    fk_joint_chain: Sequence[Mapping[str, Any]],
    model: CollisionModel,
    clearance_m: float = 0.0,
) -> CollisionResult:
    names = list(link_tf.keys())
    skip = set(adjacent_link_pairs(fk_joint_chain)) | set(model.ignore_pairs)
    worst: Optional[CollisionResult] = None
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            name_a, name_b = names[i], names[j]
            if frozenset((name_a, name_b)) in skip:
                continue
            if name_a in model.self_ignore_links or name_b in model.self_ignore_links:
                continue
            pos_a, rot_a = link_tf[name_a]
            pos_b, rot_b = link_tf[name_b]
            gap = _link_pair_gap(model, name_a, pos_a, rot_a, name_b, pos_b, rot_b) - float(clearance_m)
            if gap < 0.0:
                return CollisionResult(
                    ok=False, min_clearance_m=gap, link_a=name_a, link_b=name_b, reason="self_collision"
                )
            if worst is None or gap < worst.min_clearance_m:
                worst = CollisionResult(ok=True, min_clearance_m=gap, link_a=name_a, link_b=name_b, reason="")
    if worst is None:
        return CollisionResult(ok=True, min_clearance_m=float("inf"), link_a="", link_b="", reason="")
    return worst


def _shape_pair_gap(
    is_box: bool,
    a0_or_boxes,
    a1,
    ra,
    other_is_box: bool,
    b_center_or_p0,
    b_half_or_p1,
    b_rot_or_radius,
) -> float:
    """Dispatch the gap between one arm-link shape and one external rigid
    shape (a GO2-body shape or a static world obstacle -- both are just "a
    box or capsule fixed in some frame" from this function's point of view).

    Both sides can independently be a capsule or a box, so all four
    combinations need a real implementation, not just arm-vs-capsule.
    """
    if is_box and other_is_box:
        return min(
            box_box_gap(box_center, half_extents, box_rot, b_center_or_p0, b_half_or_p1, b_rot_or_radius)
            for box_center, half_extents, box_rot in a0_or_boxes
        )
    if is_box:
        return min(
            capsule_box_gap(b_center_or_p0, b_half_or_p1, b_rot_or_radius, box_center, half_extents, box_rot)
            for box_center, half_extents, box_rot in a0_or_boxes
        )
    if other_is_box:
        return capsule_box_gap(a0_or_boxes, a1, ra, b_center_or_p0, b_half_or_p1, b_rot_or_radius)
    return capsule_capsule_gap(a0_or_boxes, a1, ra, b_center_or_p0, b_half_or_p1, b_rot_or_radius)


def go2_leg_world_shapes(
    go2_pos: Sequence[float],
    go2_rot: np.ndarray,
    leg_q: Sequence[float],
    segments: Sequence[Go2LegSegment],
) -> list[tuple[bool, Any, Any, Any, str]]:
    """FK each leg segment (hip/thigh/calf) to its live world-space shape.

    ``leg_q`` is the flat 12-value ``[FL_hip, FL_thigh, FL_calf, FR_hip,
    ...]`` vector (matches ``Go2LegSegment.leg_q_index`` and the wire's
    ``go2_leg_q`` -- see ``elesim_model_builder.collision_model
    .build_go2_body_shapes``'s docstring for the shared ordering
    convention). Returns the same ``(is_box, ...)`` tuple shape
    ``go2_collision_check`` already uses for the base-rigid capsules/boxes,
    so both can feed the same dispatch/worst-case-tracking loop.

    Segments must be given hip-before-thigh-before-calf per leg (true of
    ``CollisionModel.go2_leg_segments`` as built) so each child's parent
    pose is already computed by the time it's needed.
    """
    go2_pos_arr = np.asarray(go2_pos, dtype=float).reshape(3)
    go2_rot_arr = np.asarray(go2_rot, dtype=float).reshape(3, 3)
    leg_q_arr = np.asarray(leg_q, dtype=float).reshape(-1)
    world_pose: dict[str, tuple[np.ndarray, np.ndarray]] = {"base": (go2_pos_arr, go2_rot_arr)}

    shapes: list[tuple[bool, Any, Any, Any, str]] = []
    for segment in segments:
        parent_pos, parent_rot = world_pose[segment.parent]
        origin_rot = np.asarray(segment.origin_rot_in_parent, dtype=float).reshape(3, 3)
        joint_angle = float(leg_q_arr[segment.leg_q_index])
        axis = np.asarray(segment.axis_in_parent, dtype=float).reshape(3)
        joint_rot = Rot.from_rotvec(axis * joint_angle).as_matrix()
        seg_pos = parent_pos + parent_rot @ np.asarray(segment.origin_from_parent, dtype=float)
        seg_rot = parent_rot @ origin_rot @ joint_rot
        world_pose[segment.name] = (seg_pos, seg_rot)

        if segment.shape_type == "box":
            center = seg_pos + seg_rot @ np.asarray(segment.center_local, dtype=float)
            box_rot = seg_rot @ np.asarray(segment.rot_local, dtype=float).reshape(3, 3)
            shapes.append((True, center, np.asarray(segment.half_extents_local, dtype=float), box_rot, segment.name))
        else:
            p0 = seg_pos + seg_rot @ np.asarray(segment.p0_local, dtype=float)
            p1 = seg_pos + seg_rot @ np.asarray(segment.p1_local, dtype=float)
            shapes.append((False, p0, p1, segment.radius, segment.name))
    return shapes


def _box_aabb_extent(half_extents: Sequence[float], rot: Sequence[Sequence[float]]) -> np.ndarray:
    """World-axis-aligned half-extent of an oriented box: for world axis i,
    sum_j |rot[i,j]| * half[j] -- the standard box-AABB identity, exact and
    cheaper than enumerating all 8 corners."""
    rot_arr = np.asarray(rot, dtype=float).reshape(3, 3)
    half_arr = np.asarray(half_extents, dtype=float).reshape(3)
    return np.abs(rot_arr) @ half_arr


def simplify_go2_to_bounding_box(model: CollisionModel, *, leg_q: Optional[Sequence[float]] = None) -> CollisionModel:
    """Replace GO2's torso/head/leg shapes with a single body-frame box
    enclosing all of them, trading the per-shape precision of checking each
    of the ~15 individual GO2 shapes (torso box, 2 head capsules, 12 leg
    segments) for one box -- go2_collision_check then does 1 shape-pair
    check per arm link instead of ~15, and never has to re-run the leg
    segments' forward kinematics (each involving a fresh
    ``Rot.from_rotvec``) at all.

    Legs genuinely move (unlike the rest of GO2's body), so this box is
    only as tight as the given ``leg_q`` snapshot -- a different stance
    needs a different box, which is exactly why this must be recomputed
    per ``generate()`` call (from that call's frozen leg_q) rather than
    baked into the shared collision_model.json once. It is also strictly
    more conservative than checking each shape individually: the merged
    box's corners can cover real empty space around/between individual leg
    segments (e.g. the gap between two standing legs), so a path that
    would have cleared a specific leg capsule can now report a collision
    against the merged box that doesn't reflect a real overlap. Accepted
    tradeoff -- see the caller for the performance motivation.

    Returns ``model`` unchanged if it has no GO2 shapes to merge (e.g. a
    fixed-base deployment with no GO2 body model at all).
    """
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)

    def _extend_box(center: Sequence[float], half_extents: Sequence[float], rot: Sequence[Sequence[float]]) -> None:
        nonlocal lo, hi
        center_arr = np.asarray(center, dtype=float).reshape(3)
        extent = _box_aabb_extent(half_extents, rot)
        lo = np.minimum(lo, center_arr - extent)
        hi = np.maximum(hi, center_arr + extent)

    def _extend_capsule(p0: Sequence[float], p1: Sequence[float], radius: float) -> None:
        nonlocal lo, hi
        p0_arr = np.asarray(p0, dtype=float).reshape(3)
        p1_arr = np.asarray(p1, dtype=float).reshape(3)
        lo = np.minimum(lo, np.minimum(p0_arr, p1_arr) - float(radius))
        hi = np.maximum(hi, np.maximum(p0_arr, p1_arr) + float(radius))

    for box in model.go2_boxes:
        _extend_box(box.center_body, box.half_extents_body, box.rot_body)
    for capsule in model.go2_capsules:
        _extend_capsule(capsule.p0_body, capsule.p1_body, capsule.radius)
    if leg_q is not None and model.go2_leg_segments:
        # Body-frame shapes: FK the leg chain from an identity "base" pose
        # rather than the real go2_pos/go2_rot, since this box is meant to
        # be combined with those the same way the existing go2_boxes are
        # (go2_collision_check applies go2_pos/go2_rot uniformly to every
        # GO2 shape).
        leg_shapes = go2_leg_world_shapes(np.zeros(3), np.eye(3), leg_q, model.go2_leg_segments)
        for is_box, s0, s1, s2, _label in leg_shapes:
            if is_box:
                _extend_box(s0, s1, s2)
            else:
                _extend_capsule(s0, s1, s2)

    if not np.all(np.isfinite(lo)):
        return model  # nothing to merge

    center = (lo + hi) / 2.0
    half_extents = (hi - lo) / 2.0
    merged_box = Go2BodyBox(
        center_body=(float(center[0]), float(center[1]), float(center[2])),
        half_extents_body=(float(half_extents[0]), float(half_extents[1]), float(half_extents[2])),
        label="go2_merged",
    )
    return replace(model, go2_boxes=(merged_box,), go2_capsules=(), go2_leg_segments=())


def go2_collision_check(
    link_tf: Mapping[str, tuple[np.ndarray, np.ndarray]],
    *,
    model: CollisionModel,
    go2_pos: Sequence[float],
    go2_rpy_rad: Sequence[float],
    leg_q: Optional[Sequence[float]] = None,
    clearance_m: float = 0.0,
) -> CollisionResult:
    if not model.go2_capsules and not model.go2_boxes and not (leg_q is not None and model.go2_leg_segments):
        return CollisionResult(ok=True, min_clearance_m=float("inf"), link_a="", link_b="", reason="")

    go2_pos_arr = np.asarray(go2_pos, dtype=float).reshape(3)
    rot = Rot.from_euler("xyz", np.asarray(go2_rpy_rad, dtype=float).reshape(3), degrees=False)
    rot_mat = rot.as_matrix()

    go2_shapes: list[tuple[bool, Any, Any, Any, str]] = []
    for capsule in model.go2_capsules:
        b0 = go2_pos_arr + rot.apply(np.asarray(capsule.p0_body, dtype=float))
        b1 = go2_pos_arr + rot.apply(np.asarray(capsule.p1_body, dtype=float))
        go2_shapes.append((False, b0, b1, capsule.radius, capsule.label or "go2_body"))
    for box in model.go2_boxes:
        center = go2_pos_arr + rot.apply(np.asarray(box.center_body, dtype=float))
        box_rot = rot_mat @ np.asarray(box.rot_body, dtype=float).reshape(3, 3)
        go2_shapes.append((True, center, np.asarray(box.half_extents_body, dtype=float), box_rot, box.label or "go2_body"))
    if leg_q is not None and model.go2_leg_segments:
        go2_shapes.extend(go2_leg_world_shapes(go2_pos_arr, rot_mat, leg_q, model.go2_leg_segments))

    worst: Optional[CollisionResult] = None
    for name, (pos, link_rot) in link_tf.items():
        if name in model.go2_ignore_links:
            continue
        is_box = model.is_box(name)
        if is_box:
            a0_or_boxes, a1, ra = model.world_boxes(name, pos, link_rot), None, None
        else:
            a0_or_boxes, a1, ra = model.world_capsule(name, pos, link_rot)
        for go2_is_box, b0_or_center, b1_or_half, b2_or_rot_or_radius, label in go2_shapes:
            gap = _shape_pair_gap(
                is_box, a0_or_boxes, a1, ra, go2_is_box, b0_or_center, b1_or_half, b2_or_rot_or_radius
            ) - float(clearance_m)
            if gap < 0.0:
                return CollisionResult(ok=False, min_clearance_m=gap, link_a=name, link_b=label, reason="go2_collision")
            if worst is None or gap < worst.min_clearance_m:
                worst = CollisionResult(ok=True, min_clearance_m=gap, link_a=name, link_b=label, reason="")
    if worst is None:
        return CollisionResult(ok=True, min_clearance_m=float("inf"), link_a="", link_b="", reason="")
    return worst


def environment_collision_check(
    link_tf: Mapping[str, tuple[np.ndarray, np.ndarray]],
    *,
    model: CollisionModel,
    clearance_m: float = 0.0,
) -> CollisionResult:
    """Check every arm link against ``model.obstacle_boxes``/``obstacle_capsules``
    (static scene obstacles, already in world coordinates -- see ``WorldBox``/
    ``WorldCapsule``)."""
    if not model.obstacle_boxes and not model.obstacle_capsules:
        return CollisionResult(ok=True, min_clearance_m=float("inf"), link_a="", link_b="", reason="")

    obstacle_shapes: list[tuple[bool, Any, Any, Any, str]] = [
        (
            True,
            np.asarray(box.center_world, dtype=float),
            np.asarray(box.half_extents_world, dtype=float),
            np.asarray(box.rot_world, dtype=float).reshape(3, 3),
            box.label or "obstacle",
        )
        for box in model.obstacle_boxes
    ] + [
        (
            False,
            np.asarray(capsule.p0_world, dtype=float),
            np.asarray(capsule.p1_world, dtype=float),
            float(capsule.radius),
            capsule.label or "obstacle",
        )
        for capsule in model.obstacle_capsules
    ]

    worst: Optional[CollisionResult] = None
    for name, (pos, link_rot) in link_tf.items():
        is_box = model.is_box(name)
        if is_box:
            a0_or_boxes, a1, ra = model.world_boxes(name, pos, link_rot), None, None
        else:
            a0_or_boxes, a1, ra = model.world_capsule(name, pos, link_rot)
        for obstacle_is_box, b0_or_center, b1_or_half, b2_or_rot, label in obstacle_shapes:
            gap = _shape_pair_gap(
                is_box, a0_or_boxes, a1, ra, obstacle_is_box, b0_or_center, b1_or_half, b2_or_rot
            ) - float(clearance_m)
            if gap < 0.0:
                return CollisionResult(
                    ok=False, min_clearance_m=gap, link_a=name, link_b=label, reason="environment_collision"
                )
            if worst is None or gap < worst.min_clearance_m:
                worst = CollisionResult(ok=True, min_clearance_m=gap, link_a=name, link_b=label, reason="")
    if worst is None:
        return CollisionResult(ok=True, min_clearance_m=float("inf"), link_a="", link_b="", reason="")
    return worst


def check_configuration(
    *,
    context: Mapping[str, Any],
    q: Sequence[float],
    model: CollisionModel,
    go2_pos: Optional[Sequence[float]] = None,
    go2_rpy_rad: Optional[Sequence[float]] = None,
    leg_q: Optional[Sequence[float]] = None,
    clearance_m: float = 0.0,
    environment_clearance_m: Optional[float] = None,
) -> CollisionResult:
    """Run self-collision, (optionally) GO2-body collision, and static
    environment-obstacle collision for one ``q``, returning whichever check
    reports the worst (or first-violated) clearance.

    ``leg_q`` (the live 12-value leg joint vector, see
    ``go2_leg_world_shapes``) is optional and independent of
    ``go2_pos``/``go2_rpy_rad``: without it, the arm is still checked
    against GO2's base-rigid shapes (torso, head), just not the legs.

    ``environment_clearance_m`` overrides ``clearance_m`` for just the
    environment-obstacle check (``None`` falls back to ``clearance_m``,
    unchanged). Self/GO2-body proxies were tuned tight against real
    self-collision incidents (see ``discover_always_colliding_pairs``) and
    should stay strict; environment obstacles are typically hand-authored
    boxes (e.g. a wall) checked against coarse, deliberately-conservative
    bounding capsules/boxes -- a link's fitted capsule radius is a tight
    bound on its *real* mesh, not a padded one, so a few mm of proxy-vs-
    proxy overlap while threading a narrow opening doesn't necessarily mean
    the real parts touch. A small negative value here (e.g. -0.01, i.e.
    tolerate up to 1cm of raw proxy overlap) absorbs that without loosening
    the self/GO2 checks that already work well at an exact 0m threshold.
    """
    # Checked cheapest/most-likely-to-reject first, not self-collision-first:
    # self-collision is O(n^2) over every link pair (~120 for this arm) while
    # environment/go2 checks are a handful of shapes per link. During RRT/
    # shortcut/maximize_clearance around a static obstacle, almost every
    # rejection is an environment (or go2) violation, not a self-collision
    # one, so paying for the O(n^2) pass first wasted it on the common path.
    # The final worst-case result is identical either way -- only the order
    # of computation (and how fast a violating q short-circuits) changes.
    link_tf = ik_kin._forward_link_tf(dict(context), q)
    env_clearance = clearance_m if environment_clearance_m is None else environment_clearance_m
    worst_result = environment_collision_check(link_tf, model=model, clearance_m=env_clearance)
    if not worst_result.ok:
        return worst_result
    if go2_pos is not None and go2_rpy_rad is not None:
        go2_result = go2_collision_check(
            link_tf, model=model, go2_pos=go2_pos, go2_rpy_rad=go2_rpy_rad, leg_q=leg_q, clearance_m=clearance_m
        )
        if not go2_result.ok:
            return go2_result
        if go2_result.min_clearance_m < worst_result.min_clearance_m:
            worst_result = go2_result
    self_result = self_collision_check(
        link_tf, fk_joint_chain=context["fk_joint_chain"], model=model, clearance_m=clearance_m
    )
    if not self_result.ok:
        return self_result
    if self_result.min_clearance_m < worst_result.min_clearance_m:
        worst_result = self_result
    return worst_result


def _violating_pairs_at(
    *,
    context: Mapping[str, Any],
    model: CollisionModel,
    q: Sequence[float],
    skip: frozenset[frozenset[str]],
) -> set[frozenset[str]]:
    link_tf = ik_kin._forward_link_tf(dict(context), q)
    names = list(link_tf.keys())
    bad: set[frozenset[str]] = set()
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            name_a, name_b = names[i], names[j]
            key = frozenset((name_a, name_b))
            if key in skip:
                continue
            if name_a in model.self_ignore_links or name_b in model.self_ignore_links:
                continue
            pos_a, rot_a = link_tf[name_a]
            pos_b, rot_b = link_tf[name_b]
            if _link_pair_gap(model, name_a, pos_a, rot_a, name_b, pos_b, rot_b) < 0.0:
                bad.add(key)
    return bad


def discover_always_colliding_pairs(
    *,
    context: Mapping[str, Any],
    model: CollisionModel,
    num_samples: int = 300,
    seed: int = 0,
    extra_q_samples: Sequence[Sequence[float]] = (),
    min_fraction: float = 1.0,
) -> frozenset[frozenset[str]]:
    """Sample joint configurations and find link pairs that (almost) never clear.

    Mirrors the technique MoveIt's Setup Assistant uses to build a default
    self-collision matrix: a pair that reports overlap across (at least)
    ``min_fraction`` of sampled poses is built too close together to ever
    give a useful collision signal (as opposed to a real, pose-dependent
    hazard), so it should be excluded rather than reject every planned
    move. Already-adjacent and already-ignored pairs are skipped; this only
    widens the exclusion set, it never re-includes a pair.

    ``min_fraction`` defaults to a strict 1.0 (every single sample must
    violate), but a handful of arbitrary reference poses -- e.g. the
    solver's named neutral/bent IK seeds, which are numerically convenient
    optimizer starting points, not verified collision-free targets -- can
    each clear a pair that is otherwise in permanent violation everywhere
    else. Callers mixing such fixed poses into ``extra_q_samples`` should
    pass a slightly relaxed threshold (e.g. ``0.98``) so one atypical pose
    doesn't mask an otherwise-consistent signal from the random sweep.

    ``extra_q_samples`` are always checked in addition to the random sweep:
    uniform random sampling essentially never lands exactly on a specific
    reference pose, so a pair that only collides at that one boundary
    configuration would otherwise slip through undetected.
    """
    reach_model = ik_kin._ReachModel(context=dict(context), limit=context["limit"])
    rng = np.random.default_rng(seed)
    fk_chain = context["fk_joint_chain"]
    skip = adjacent_link_pairs(fk_chain) | set(model.ignore_pairs)

    all_samples: list[np.ndarray] = [np.asarray(q, dtype=float).reshape(4) for q in extra_q_samples]
    all_samples.extend(
        np.array(
            [
                rng.uniform(reach_model.linear_min, reach_model.linear_max),
                rng.uniform(reach_model.roll_min, reach_model.roll_max),
                rng.uniform(-reach_model.bend_lim, reach_model.bend_lim),
                rng.uniform(-reach_model.bend_lim, reach_model.bend_lim),
            ]
        )
        for _ in range(int(num_samples))
    )
    if not all_samples:
        return frozenset()

    violation_counts: dict[frozenset[str], int] = {}
    for q in all_samples:
        for key in _violating_pairs_at(context=context, model=model, q=q, skip=skip):
            violation_counts[key] = violation_counts.get(key, 0) + 1

    threshold = float(np.clip(min_fraction, 0.0, 1.0)) * len(all_samples)
    return frozenset(key for key, count in violation_counts.items() if count >= threshold - 1e-9)


__all__ = [
    "CollisionModel",
    "CollisionResult",
    "Go2BodyBox",
    "Go2BodyCapsule",
    "Go2LegSegment",
    "LinkBox",
    "LinkCapsule",
    "adjacent_link_pairs",
    "box_box_gap",
    "capsule_box_gap",
    "capsule_capsule_gap",
    "check_configuration",
    "closest_point_on_segment",
    "discover_always_colliding_pairs",
    "environment_collision_check",
    "go2_collision_check",
    "go2_leg_world_shapes",
    "load_collision_model_for_config",
    "segment_box_distance",
    "point_segment_distance",
    "segment_segment_distance",
    "self_collision_check",
    "simplify_go2_to_bounding_box",
    "WorldBox",
    "WorldCapsule",
]
