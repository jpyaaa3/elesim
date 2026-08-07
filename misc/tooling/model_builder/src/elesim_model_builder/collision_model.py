"""Build the static collision-proxy geometry artifact from source mesh assets.

Generates the sibling artifact to ``arm_model.json`` consumed by
``elesim_controller.robot.arm.planning.collision.CollisionModel``: a
bounding-capsule (segment + radius) per arm link, derived from that link's
own mesh vertex extents, plus optional GO2-chassis capsules.

A capsule rather than a single bounding sphere: several base-assembly parts
are long and thin, and a sphere centered anywhere on them has to cover
their *entire* length -- which, measured against the real mesh geometry,
made every tested arm pose report a false self-collision against
neighbouring links. A capsule's radius only has to cover the part's
cross-section, so it stays tight for elongated parts while still closely
bounding compact ones (a capsule's radius is provably <= the equivalent
bounding sphere's, since it only measures the perpendicular component of
each vertex's offset from the capsule axis).

Two base-assembly parts (``plate``, ``housing``) are flat/wide rather than
elongated: even with a correctly-fit capsule (see ``chain_axis_capsule``'s
docstring for the degenerate-fit bug this replaced), a single radius has
to cover their *widest* cross-section, which measured against a fresh
random-pose sweep still overlapped a large, smoothly-decaying fraction of
the arm's reachable range -- from 100% at the immediate neighbours down to
~11-28% even at the gripper tip, with no natural cutoff to hand-pick
ignore-pairs around. Those two are fit as an oriented box instead (see
``bounding_box``/``LinkBox``), whose three independent half-extents match
a flat part's actual thin cross-section.

``plate``'s mesh additionally has a small (~6% of vertices) outlier
cluster far from its main body along local X, joined to it only by a
handful of grossly oversized triangles (confirmed by measuring every
face's area) -- a mesh-export defect, not a real second sub-feature.
``bounding_box_excluding_outlier_cluster`` fits the box to the main
cluster only and discards the rest, rather than trusting a box fit to
that cluster's own (equally suspect) geometry.

GO2 body dimensions are never guessed: by default they're read straight out
of the real collision geometry the Simulator itself already loads
(``<assets>/go2/go2.urdf``'s ``base`` link collision box). Explicit
``--go2-body-length/-width/-height`` still override that when given, and if
no URDF is found and no override is given, the artifact ships with no GO2
capsules and ``go2_collision_check`` reports every configuration as clear
of the body.

When a ``config`` path is given, this also runs
``elesim_controller.robot.arm.planning.collision.discover_always_colliding_pairs``
(the same random-pose-sampling technique MoveIt's Setup Assistant uses to
build a default self-collision matrix) and folds any newly-discovered
always-overlapping pair into ``ignore_pairs`` automatically, instead of
requiring each one to be found and added by hand.
"""

from __future__ import annotations

import json
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rot

DEFAULT_GO2_URDF_LINK_NAME = "base"

DEFAULT_IGNORE_PAIRS: tuple[tuple[str, str], ...] = (("gripper_claw_left", "gripper_claw_right"),)
PLATE_LINK_NAME = "plate"
# `housing`/gripper parts are fit as boxes (see module docstring) rather
# than wholesale-excluded from *self*-collision; any pair that still
# always-overlaps another arm link by construction (e.g. an immediate
# chain neighbour) is caught by `discover_always_colliding_pairs` below and
# folded into `ignore_pairs`.
#
# `plate` is deliberately NOT in this list -- excluded from collision
# checking entirely instead (see DEFAULT_SELF_IGNORE_LINKS/
# DEFAULT_GO2_IGNORE_LINKS below). Its mesh has real, physically distinct
# sub-features (a thin main body, a thin bridge, and a raised Jetson Orin
# mount -- see `fit_plate_link_boxes`, still available and tested for
# anyone revisiting this) at very different scales along its own length,
# which a rigid box (or three) fits poorly enough in practice -- confirmed
# live, an oriented 3-box fit still rendered wrong/misleading relative to
# the actual mesh -- that it's not worth the maintenance cost right now.
BOX_FIT_LINK_NAMES: tuple[str, ...] = (
    "housing",
    "gripper_base",
    "gripper_claw_left",
    "gripper_claw_right",
)
# `plate`/`housing` are both excluded from the GO2-body check: `plate` is
# bolted directly onto GO2's back and `housing` sits on `plate` -- both are
# *meant* to sit at/overlap the chassis surface by construction (confirmed
# live: the arm's ordinary resting pose reports `plate` vs `go2_chassis` at
# -0.037m, i.e. always in "collision" with the body it's mounted to).
# That's a mounting, not a hazard.
DEFAULT_GO2_IGNORE_LINKS: tuple[str, ...] = ("plate", "housing")
# Self-collision (vs the arm's *other* links): `plate` alone stays
# excluded here too (see BOX_FIT_LINK_NAMES's comment on why); `housing`
# and the gripper parts are checked normally now that they're box-fit.
DEFAULT_SELF_IGNORE_LINKS: tuple[str, ...] = ("plate",)


def _parse_obj_vertices(path: Path) -> np.ndarray:
    vertices: list[list[float]] = []
    with open(path, "r", encoding="utf-8") as handle:
        for line in handle:
            if not line.startswith("v "):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not vertices:
        raise RuntimeError(f"mesh has no vertices: {path}")
    return np.asarray(vertices, dtype=float)


def bounding_capsule(path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    """Tight capsule fit: axis along the mesh's longest AABB extent.

    ``radius`` is the farthest any vertex sits from that axis line (the
    *perpendicular* component of its offset from the AABB midpoint), not
    from a single center point -- the axial component is absorbed by the
    segment's own length instead of inflating the radius.

    Only a mesh-shape heuristic -- it has no idea which direction the FK
    chain actually advances in. Prefer ``chain_axis_capsule`` (below) for
    any link with a known child offset; this is the fallback for leaf
    links (no child in the 4-DOF chain) where that offset doesn't exist.
    """
    vertices = _parse_obj_vertices(path)
    lo = vertices.min(axis=0)
    hi = vertices.max(axis=0)
    center = (lo + hi) / 2.0
    extents = hi - lo
    axis_idx = int(np.argmax(extents))
    axis = np.zeros(3, dtype=float)
    axis[axis_idx] = 1.0
    half_length = float(extents[axis_idx]) / 2.0
    p0 = center - half_length * axis
    p1 = center + half_length * axis

    offsets = vertices - center.reshape(1, 3)
    axial = offsets @ axis
    perpendicular = offsets - np.outer(axial, axis)
    radius = float(np.max(np.linalg.norm(perpendicular, axis=1)))
    return p0, p1, radius


def chain_axis_capsule(path: Path, axis_local: Sequence[float]) -> tuple[np.ndarray, np.ndarray, float]:
    """Capsule fit aligned to the FK chain's own travel direction, not the mesh's AABB.

    ``axis_local`` is this link's own-frame offset to its child (a
    ``fk_joint_chain`` entry's ``origin_parent``) -- the direction the arm
    physically advances by one segment, which is *not* necessarily the
    mesh's longest AABB extent. The continuum "node" parts are the
    concrete case that breaks the AABB heuristic: each is a short, wide
    disc -- ~0.05m along the chain (X) but ~0.088m across its own
    revolute axis (Y, the mesh's longest extent) -- so ``bounding_capsule``
    orients the capsule sideways to the chain instead of along it. That
    silently under-covers self-intersection when several such discs curl
    the same way (confirmed against the running Simulator: a live pose at
    theta1=32d/theta2=26d visibly self-intersected while the old capsule
    fit reported +3.8cm clearance).

    Vertices are assumed given in this part's own frame with the origin at
    its own joint pivot (true for every part in this model -- e.g. the node
    mesh's local X spans exactly 0..0.05, matching ``origin_parent``, not
    a mesh-centered range), so ``p0_local`` is that origin and ``p1_local``
    is the given offset; radius is the farthest any vertex sits from the
    infinite line through them.

    A zero-length ``axis_local`` (the child sits at this link's own origin,
    e.g. ``plate``) means there is no chain direction to align to -- falls
    back to ``bounding_capsule``, which fits an actual two-point capsule
    along the mesh's own longest extent. Collapsing to a single-point
    sphere measured from this link's local origin is wrong whenever that
    origin isn't near the mesh's own centroid (``plate``'s sits at one
    edge, not the middle): confirmed live, this produced a 0.555m-radius
    sphere for `plate` -- big enough to swallow the tip at any nearby
    intermediate RRT waypoint without the checker ever flagging it.
    """
    vertices = _parse_obj_vertices(path)
    axis_vec = np.asarray(axis_local, dtype=float).reshape(3)
    length = float(np.linalg.norm(axis_vec))
    if length <= 1e-9:
        return bounding_capsule(path)
    origin = np.zeros(3, dtype=float)
    axis_unit = axis_vec / length
    axial = vertices @ axis_unit
    perpendicular = vertices - np.outer(axial, axis_unit)
    radius = float(np.max(np.linalg.norm(perpendicular, axis=1)))
    return origin, axis_vec.copy(), radius


def bounding_box(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Axis-aligned (in this part's own local frame) bounding box: (center, half_extents).

    For a flat/wide part (``plate``, ``housing``) a capsule's single
    radius has to cover the widest cross-section, over-approximating the
    thin dimension enough to falsely overlap a large fraction of the arm's
    own reachable range (see this module's docstring). A box's three
    independent half-extents fit each axis separately instead.
    """
    vertices = _parse_obj_vertices(path)
    lo = vertices.min(axis=0)
    hi = vertices.max(axis=0)
    center = (lo + hi) / 2.0
    half_extents = (hi - lo) / 2.0
    return center, half_extents


def bounding_box_pair_by_largest_gap(path: Path, *, axis: int = 0) -> tuple[
    tuple[np.ndarray, np.ndarray], tuple[np.ndarray, np.ndarray]
]:
    """Split a mesh's vertices at the largest gap along ``axis`` and fit a box to each half.

    ``t -> vertex_position[axis]`` sorted is a 1D point set; the largest
    gap between consecutive sorted values is *the* natural split point
    between two well-separated clusters (no clustering hyperparameters
    needed for a two-cluster split). Returns ``(main_box, secondary_box)``,
    where ``main_box`` is fit to the larger of the two vertex groups --
    order is otherwise arbitrary, both are checked identically once loaded.

    Low-level primitive -- see ``bounding_box_excluding_outlier_cluster``
    for the actual use on ``plate`` (which discards ``secondary_box``
    rather than keeping it; that cluster turned out to be a mesh-export
    defect, not a real second sub-feature -- see that function's
    docstring).
    """
    vertices = _parse_obj_vertices(path)
    order = np.argsort(vertices[:, axis])
    sorted_axis_vals = vertices[order, axis]
    gaps = np.diff(sorted_axis_vals)
    split = int(np.argmax(gaps)) + 1
    group_a = vertices[order[:split]]
    group_b = vertices[order[split:]]

    def _box(group: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        lo = group.min(axis=0)
        hi = group.max(axis=0)
        return (lo + hi) / 2.0, (hi - lo) / 2.0

    box_a, box_b = _box(group_a), _box(group_b)
    return (box_a, box_b) if len(group_a) >= len(group_b) else (box_b, box_a)


def bounding_box_excluding_outlier_cluster(path: Path, *, axis: int = 0) -> tuple[np.ndarray, np.ndarray]:
    """Fit a box to a mesh's main vertex cluster, discarding a minority outlier cluster.

    ``plate``'s mesh has ~6% of its vertices (52 of 820) sitting far away
    along local X from the other 94%, connected to the main body only by a
    handful of *enormous* triangles (up to ~6400x the mesh's median face
    area, confirmed by measuring every face) -- a mesh-export defect (a
    stray/erroneous bridge), not a legitimate second sub-feature. A single
    ``bounding_box()`` inflates across that entire gap, which (measured
    live) falsely overlapped the arm across a much larger fraction of its
    reachable range than the real, compact plate would.

    Rather than fit a second box to that outlier cluster (its own geometry
    can't be trusted either, being defined by those same oversized/likely
    corrupted faces), it's discarded outright: anything only reaching into
    that region isn't flagged as a collision at all. That's a real,
    accepted gap in coverage until a corrected mesh asset is available --
    not a proxy-fit compromise.
    """
    main_box, _discarded = bounding_box_pair_by_largest_gap(path, axis=axis)
    return main_box


def fit_plate_link_boxes(path: Path) -> list[tuple[np.ndarray, np.ndarray]]:
    """Fit three boxes to ``plate``'s mesh: main body, thin bridge, and Jetson Orin.

    Confirmed live (collision-geometry overlay against the running
    Simulator) that ``plate`` is three real, physically distinct
    sub-features, not one: a thin main body near the arm's base, a thin
    bridging plate extending back from it, and a raised compute module
    (Jetson Orin) mounted at the bridge's far end. Splitting is done in two
    stages since the two splits aren't along the same axis:

    1. ``bounding_box_pair_by_largest_gap`` along local X separates the
       main body (the dense majority cluster) from everything further back
       (bridge + Orin, both sparse and thin -- this is the same split
       ``bounding_box_excluding_outlier_cluster`` uses, just keeping the
       second cluster instead of discarding it).
    2. That remaining cluster is split again, this time by height (Z):
       vertices at or above the gap found between this mesh's two
       Z-clusters are the raised Orin block sitting on top; everything
       below is the thin bridge running underneath and past it.
    """
    vertices = _parse_obj_vertices(path)
    order = np.argsort(vertices[:, 0])
    sorted_x = vertices[order, 0]
    gaps = np.diff(sorted_x)
    split = int(np.argmax(gaps)) + 1
    # `bounding_box_pair_by_largest_gap` orders its result by vertex count
    # (main = more vertices); reproduce that here directly since this
    # function needs the index sets themselves, not just the fitted boxes.
    if len(order[:split]) >= len(order[split:]):
        main_idx, tail_idx = order[:split], order[split:]
    else:
        main_idx, tail_idx = order[split:], order[:split]
    main = vertices[main_idx]
    tail = vertices[tail_idx]

    tail_order = np.argsort(tail[:, 2])
    sorted_z = tail[tail_order, 2]
    z_gaps = np.diff(sorted_z)
    z_split = int(np.argmax(z_gaps)) + 1
    bridge = tail[tail_order[:z_split]]
    orin = tail[tail_order[z_split:]]

    def _box(group: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        lo = group.min(axis=0)
        hi = group.max(axis=0)
        return (lo + hi) / 2.0, (hi - lo) / 2.0

    bridge_center, bridge_half = _box(bridge)
    orin_center, orin_half = _box(orin)
    # This mesh only has vertices exactly at the Orin block's *top* face (no
    # side-wall tessellation between the bridge's top and the Orin's top),
    # so fitting a box to `orin`'s own vertices alone gives a zero-thickness
    # slice sitting right at the top -- leaving a real, uncovered gap
    # between the bridge's top and the Orin's bottom. Extend the Orin box
    # down to meet the bridge's top face instead, so the two boxes share a
    # boundary with no dead zone between them.
    bridge_top_z = bridge_center[2] + bridge_half[2]
    orin_top_z = orin_center[2] + orin_half[2]
    orin_center = np.array([orin_center[0], orin_center[1], (bridge_top_z + orin_top_z) / 2.0])
    orin_half = np.array([orin_half[0], orin_half[1], (orin_top_z - bridge_top_z) / 2.0])

    return [_box(main), (bridge_center, bridge_half), (orin_center, orin_half)]


def _single_child_offset(link_name: str, fk_joint_chain: Sequence[Any]) -> Optional[np.ndarray]:
    """This link's own-frame offset to its child, iff it has exactly one.

    Branching links (e.g. ``gripper_base``, with claw and camera children)
    and leaf links (no child in the 4-DOF chain, e.g. the claws/camera
    themselves) return ``None`` -- there's no single unambiguous "chain
    travel direction" for them, so callers should fall back to
    ``bounding_capsule``.
    """
    matches = [meta for meta in fk_joint_chain if str(meta["parent"]) == link_name]
    if len(matches) != 1:
        return None
    return np.asarray(matches[0]["origin_parent"], dtype=float).reshape(3)


def parse_urdf_link_collision_box(urdf_path: Path, *, link_name: str = DEFAULT_GO2_URDF_LINK_NAME) -> tuple[float, float, float]:
    """Read a named link's ``<collision><geometry><box size="x y z"/>`` from a URDF.

    Used to pull GO2's real chassis dimensions straight out of
    ``assets/go2/go2.urdf`` -- the same file the Simulator already loads to
    build the runtime GO2 model -- instead of asking the caller to guess or
    hand-supply body dimensions that are already sitting in the repo.
    """
    root = ET.parse(urdf_path).getroot()
    for link in root.findall("link"):
        if link.get("name") != link_name:
            continue
        box = link.find("./collision/geometry/box")
        if box is None:
            raise RuntimeError(f"link '{link_name}' in {urdf_path} has no <collision><box> geometry")
        size_raw = str(box.get("size", "")).split()
        if len(size_raw) != 3:
            raise RuntimeError(f"link '{link_name}' collision box in {urdf_path} must have 3 size values")
        return tuple(float(x) for x in size_raw)
    raise RuntimeError(f"link '{link_name}' not found in {urdf_path}")


GO2_LEG_NAMES: tuple[str, ...] = ("FL", "FR", "RL", "RR")
GO2_LEG_PARTS: tuple[str, ...] = ("hip", "thigh", "calf")


def _parse_urdf_origin(origin_el: Optional[ET.Element]) -> tuple[np.ndarray, np.ndarray]:
    """A URDF ``<origin xyz="..." rpy="...">`` (or its absence, = identity) as (xyz, 3x3 rotation)."""
    xyz_raw = origin_el.get("xyz", "0 0 0") if origin_el is not None else "0 0 0"
    rpy_raw = origin_el.get("rpy", "0 0 0") if origin_el is not None else "0 0 0"
    xyz = np.array([float(x) for x in xyz_raw.split()], dtype=float)
    rpy = np.array([float(x) for x in rpy_raw.split()], dtype=float)
    rot = Rot.from_euler("xyz", rpy).as_matrix()
    return xyz, rot


def _parse_urdf_link_collision_shape(link_el: ET.Element) -> Optional[dict[str, Any]]:
    """A link's ``<collision>`` geometry (box/cylinder/sphere), in the link's own frame.

    ``None`` if the link has no ``<collision>`` element (e.g. ``imu``/
    ``radar``, which exist in the URDF purely for sensor placement).
    Cylinders/spheres become a capsule (p0_local/p1_local/radius); a sphere
    is the degenerate zero-length case. Boxes stay boxes (center_local/
    half_extents_local/rot_local) -- there's no reason to approximate a
    genuinely box-shaped link (e.g. the torso, or a leg's thigh) as a
    capsule when this collision model already supports boxes directly.
    """
    collision = link_el.find("collision")
    if collision is None:
        return None
    origin_xyz, origin_rot = _parse_urdf_origin(collision.find("origin"))
    geometry = collision.find("geometry")
    shape_el = next(iter(geometry), None)
    if shape_el is None:
        return None
    if shape_el.tag == "box":
        size = np.array([float(x) for x in shape_el.get("size", "").split()], dtype=float)
        return {
            "shape_type": "box",
            "center_local": origin_xyz,
            "half_extents_local": size / 2.0,
            "rot_local": origin_rot,
        }
    if shape_el.tag == "cylinder":
        radius = float(shape_el.get("radius", "0"))
        length = float(shape_el.get("length", "0"))
        half_axis = origin_rot @ np.array([0.0, 0.0, length / 2.0])
        return {
            "shape_type": "capsule",
            "p0_local": origin_xyz - half_axis,
            "p1_local": origin_xyz + half_axis,
            "radius": radius,
        }
    if shape_el.tag == "sphere":
        radius = float(shape_el.get("radius", "0"))
        return {"shape_type": "capsule", "p0_local": origin_xyz, "p1_local": origin_xyz, "radius": radius}
    raise RuntimeError(f"unsupported <collision> geometry tag: {shape_el.tag!r}")


def build_go2_body_shapes(urdf_path: Path) -> dict[str, Any]:
    """GO2's real collision geometry from its URDF: base-rigid shapes plus per-leg segments.

    Two categories, since they need very different runtime handling:

    - Base-rigid (``go2_capsules``/``go2_boxes``): ``base`` itself plus any
      link reachable from it through only ``fixed`` joints (``Head_upper``,
      ``Head_lower`` -- confirmed both attach via ``type="fixed"`` joints,
      so their pose relative to ``base`` never changes and needs no leg
      telemetry). Each contributes its own capsule or box, in GO2's base
      frame, exactly like the base link's own collision box already did.

    - Leg segments (``go2_leg_segments``): each leg's hip/thigh/calf attach
      via ``revolute`` joints and genuinely move as GO2 walks, so their
      world pose depends on the live joint angles (``go2_leg_q``, already
      flowing controller-side via ``HostState`` -- see
      ``elesim_controller.robot.arm.planning.collision.go2_leg_world_shapes``
      for the FK that consumes this). Ordered ``FL,FR,RL,RR`` x
      ``hip,thigh,calf`` to match ``Go2KinematicsModel.all_leg_dof_idx``'s
      iteration order (the same order the wire's ``go2_leg_q`` 12-tuple
      already uses elsewhere in this codebase) -- ``leg_q_index`` in each
      entry is that flat 0..11 index, not re-derived at runtime.

    Deliberately reads every value straight from the URDF rather than
    assuming left/right symmetry: the four legs' hip/thigh dimensions do
    mirror exactly, but ``FL_calf``'s collision cylinder is very slightly
    different from the other three (0.012m vs 0.013m radius, etc.) --
    presumably a real, small hardware asymmetry -- which a hand-written
    "mirror FL" formula would have silently gotten wrong for 3 of 4 legs.
    """
    root = ET.parse(urdf_path).getroot()
    links_by_name = {str(link.get("name")): link for link in root.findall("link")}
    joints_by_child: dict[str, dict[str, Any]] = {}
    for joint in root.findall("joint"):
        child = str(joint.find("child").get("link"))
        parent = str(joint.find("parent").get("link"))
        origin_xyz, origin_rot = _parse_urdf_origin(joint.find("origin"))
        axis_el = joint.find("axis")
        axis_raw = axis_el.get("xyz", "0 0 0") if axis_el is not None else "0 0 0"
        joints_by_child[child] = {
            "parent": parent,
            "type": str(joint.get("type")),
            "origin_xyz": origin_xyz,
            "origin_rot": origin_rot,
            "axis": np.array([float(x) for x in axis_raw.split()], dtype=float),
        }

    go2_capsules: list[dict[str, Any]] = []
    go2_boxes: list[dict[str, Any]] = []

    def _collect_base_rigid(link_name: str, cum_pos: np.ndarray, cum_rot: np.ndarray) -> None:
        shape = _parse_urdf_link_collision_shape(links_by_name[link_name])
        if shape is not None:
            if shape["shape_type"] == "box":
                go2_boxes.append(
                    {
                        "center_body": (cum_pos + cum_rot @ shape["center_local"]).tolist(),
                        "half_extents_body": shape["half_extents_local"].tolist(),
                        "rot_body": (cum_rot @ shape["rot_local"]).tolist(),
                        "label": link_name,
                    }
                )
            else:
                go2_capsules.append(
                    {
                        "p0_body": (cum_pos + cum_rot @ shape["p0_local"]).tolist(),
                        "p1_body": (cum_pos + cum_rot @ shape["p1_local"]).tolist(),
                        "radius": shape["radius"],
                        "label": link_name,
                    }
                )
        for child, info in joints_by_child.items():
            if info["parent"] == link_name and info["type"] == "fixed":
                child_pos = cum_pos + cum_rot @ info["origin_xyz"]
                child_rot = cum_rot @ info["origin_rot"]
                _collect_base_rigid(child, child_pos, child_rot)

    _collect_base_rigid("base", np.zeros(3), np.eye(3))

    leg_segments: list[dict[str, Any]] = []
    leg_q_index = 0
    for leg in GO2_LEG_NAMES:
        parent_name = "base"
        for part in GO2_LEG_PARTS:
            link_name = f"{leg}_{part}"
            info = joints_by_child[link_name]
            if info["type"] != "revolute":
                raise RuntimeError(f"expected '{link_name}' joint to be revolute, got {info['type']!r}")
            shape = _parse_urdf_link_collision_shape(links_by_name[link_name])
            if shape is None:
                raise RuntimeError(f"leg link '{link_name}' has no <collision> geometry")
            entry: dict[str, Any] = {
                "name": link_name,
                "parent": parent_name,
                "origin_from_parent": info["origin_xyz"].tolist(),
                "origin_rot_in_parent": info["origin_rot"].tolist(),
                "axis_in_parent": info["axis"].tolist(),
                "leg_q_index": leg_q_index,
                "shape_type": shape["shape_type"],
            }
            if shape["shape_type"] == "box":
                entry["center_local"] = shape["center_local"].tolist()
                entry["half_extents_local"] = shape["half_extents_local"].tolist()
                entry["rot_local"] = shape["rot_local"].tolist()
            else:
                entry["p0_local"] = shape["p0_local"].tolist()
                entry["p1_local"] = shape["p1_local"].tolist()
                entry["radius"] = shape["radius"]
            leg_segments.append(entry)
            parent_name = link_name
            leg_q_index += 1

    return {"go2_capsules": go2_capsules, "go2_boxes": go2_boxes, "go2_leg_segments": leg_segments}


def build_link_capsules(
    *,
    blueprint: dict[str, Any],
    source_root: Path,
    fk_joint_chain: Optional[Sequence[Any]] = None,
    box_fit_names: Sequence[str] = (),
) -> dict[str, dict[str, Any]]:
    """Per-link bounding capsule, keyed by blueprint part name.

    Part names match the ``fk_joint_chain`` link names in ``arm_model.json``
    one-to-one (both are sourced from the same blueprint). Endpoints are in
    each part's own local frame, to be transformed by that link's FK pose
    at check time.

    When ``fk_joint_chain`` is given, a link with exactly one child is
    fit with ``chain_axis_capsule`` (aligned to the chain's real travel
    direction) rather than ``bounding_capsule`` (mesh AABB heuristic,
    which orients the continuum "node" capsules sideways -- see
    ``chain_axis_capsule``'s docstring). Branching/leaf links, and any
    call with no chain given, still use the AABB fallback.

    Names in ``box_fit_names`` are skipped entirely -- those get a box fit
    instead, see ``build_link_boxes``.
    """
    box_fit = frozenset(box_fit_names)
    capsules: dict[str, dict[str, Any]] = {}
    for part in blueprint.get("parts", []):
        name = str(part.get("name", "")).strip()
        if not name or name in box_fit:
            continue
        mesh_rel = str((part.get("assets") or {}).get("mesh", "")).strip()
        if not mesh_rel:
            continue
        mesh_path = source_root / mesh_rel
        child_offset = _single_child_offset(name, fk_joint_chain) if fk_joint_chain is not None else None
        if child_offset is not None:
            p0, p1, radius = chain_axis_capsule(mesh_path, child_offset)
        else:
            p0, p1, radius = bounding_capsule(mesh_path)
        capsules[name] = {
            "p0_local": [float(x) for x in p0],
            "p1_local": [float(x) for x in p1],
            "radius": radius,
        }
    return capsules


def build_link_boxes(
    *,
    blueprint: dict[str, Any],
    source_root: Path,
    box_fit_names: Sequence[str],
) -> dict[str, list[dict[str, Any]]]:
    """One or more bounding boxes per link, keyed by blueprint part name, for ``box_fit_names`` only.

    ``PLATE_LINK_NAME`` gets three boxes (see ``fit_plate_link_boxes``);
    every other box-fit name gets a plain ``bounding_box()`` fit.
    """
    box_fit = frozenset(box_fit_names)
    boxes: dict[str, list[dict[str, Any]]] = {}
    for part in blueprint.get("parts", []):
        name = str(part.get("name", "")).strip()
        if not name or name not in box_fit:
            continue
        mesh_rel = str((part.get("assets") or {}).get("mesh", "")).strip()
        if not mesh_rel:
            continue
        mesh_path = source_root / mesh_rel
        if name == PLATE_LINK_NAME:
            fits = fit_plate_link_boxes(mesh_path)
        else:
            fits = [bounding_box(mesh_path)]
        boxes[name] = [
            {"center_local": [float(x) for x in center], "half_extents_local": [float(x) for x in half_extents]}
            for center, half_extents in fits
        ]
    return boxes


def _build_context(*, config_path: Path, source_root: Path) -> dict[str, Any]:
    """Build the same solver context the real Controller/RRT planner uses.

    Needed for two things: discovering always-colliding pairs below, and
    (in ``build_collision_model``) giving ``build_link_capsules`` the real
    ``fk_joint_chain`` so it can fit capsules along each link's actual
    chain-travel direction instead of guessing from mesh AABB alone.
    """
    from elesim_model_builder import json_builder
    from elesim_model_builder.context_builder import build_solver_context

    previous_asset_root = json_builder.DEFAULT_ASSET_ROOT_DIR
    try:
        # DEFAULT_ASSET_ROOT_DIR must point at the `assets/` dir itself, not
        # its parent -- matches build_arm_model's convention exactly.
        json_builder.DEFAULT_ASSET_ROOT_DIR = str((source_root / "assets").resolve())
        with tempfile.TemporaryDirectory(prefix="elesim-collision-model-") as workspace:
            # os.path.realpath: on macOS TMPDIR sits under the /var -> /private/var
            # symlink, and context_builder computes relative asset paths with plain
            # os.path.relpath -- an unresolved workspace path makes those relative
            # paths one directory level short (see misc/tooling/model_builder's
            # pre-existing test_arm_model.py failure on macOS, same root cause).
            _bundle, context = build_solver_context(str(config_path), build_dir=os.path.realpath(workspace))
    finally:
        json_builder.DEFAULT_ASSET_ROOT_DIR = previous_asset_root
    return context


def _discover_extra_ignore_pairs(
    *,
    context: dict[str, Any],
    link_capsules: dict[str, dict[str, Any]],
    link_boxes: dict[str, dict[str, Any]],
    default_radius: float,
    ignore_pairs: frozenset[frozenset[str]],
    self_ignore_links: frozenset[str],
    discovery_samples: int,
) -> frozenset[frozenset[str]]:
    from elesim_controller.robot.arm.iklib.kinematics import Q_BENT, Q_NEUTRAL
    from elesim_controller.robot.arm.planning.collision import CollisionModel, discover_always_colliding_pairs

    model = CollisionModel.from_dict(
        {
            "schema_version": 1,
            "link_capsules": link_capsules,
            "link_boxes": link_boxes,
            "default_radius": default_radius,
            "ignore_pairs": [list(pair) for pair in ignore_pairs],
            "self_ignore_links": list(self_ignore_links),
        }
    )
    return discover_always_colliding_pairs(
        context=context,
        model=model,
        num_samples=discovery_samples,
        extra_q_samples=(Q_NEUTRAL, Q_BENT),
        # Q_NEUTRAL/Q_BENT are numerically convenient IK optimizer seeds,
        # not verified collision-free targets -- one of them clearing a
        # pair that is otherwise in near-permanent violation across the
        # random sweep should not, by itself, suppress that real signal.
        min_fraction=0.98,
    )


def build_wall_with_hole_boxes(
    *,
    center_world: Sequence[float],
    width_m: float,
    height_m: float,
    thickness_m: float,
    hole_width_m: float,
    hole_height_m: float,
    hole_offset_m: Sequence[float] = (0.0, 0.0),
    label_prefix: str = "wall",
) -> list[dict[str, Any]]:
    """Build a rectangular wall obstacle with a rectangular hole in it, as a
    "picture frame" of up to 4 non-overlapping axis-aligned boxes.

    The wall spans world Y (``width_m``) and Z (``height_m``), is
    ``thickness_m`` thick along X, and is centered at ``center_world``. The
    hole is centered at ``center_world``'s Y/Z plus ``hole_offset_m`` --
    pass a non-zero offset to place it off-center. A box with non-positive
    extent (the hole touching an edge) is silently dropped rather than
    emitted as a degenerate/inverted box.

    Returns a list of dicts in the exact shape
    ``elesim_controller.robot.arm.planning.collision.CollisionModel``'s
    ``obstacle_boxes`` expects (``center_world``/``half_extents_world``/
    ``rot_world``/``label``), ready to drop into a collision-model JSON's
    ``"obstacle_boxes"`` list.
    """
    cx, cy, cz = (float(x) for x in center_world)
    hole_off_y, hole_off_z = (float(x) for x in hole_offset_m)
    half_x = float(thickness_m) / 2.0

    wall_y_min, wall_y_max = cy - float(width_m) / 2.0, cy + float(width_m) / 2.0
    wall_z_min, wall_z_max = cz - float(height_m) / 2.0, cz + float(height_m) / 2.0
    hole_cy, hole_cz = cy + hole_off_y, cz + hole_off_z
    hole_y_min, hole_y_max = hole_cy - float(hole_width_m) / 2.0, hole_cy + float(hole_width_m) / 2.0
    hole_z_min, hole_z_max = hole_cz - float(hole_height_m) / 2.0, hole_cz + float(hole_height_m) / 2.0

    identity_rot = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    def _bar(y_min: float, y_max: float, z_min: float, z_max: float, label: str) -> Optional[dict[str, Any]]:
        half_y = (y_max - y_min) / 2.0
        half_z = (z_max - z_min) / 2.0
        if half_y <= 1e-9 or half_z <= 1e-9:
            return None
        return {
            "center_world": [cx, (y_min + y_max) / 2.0, (z_min + z_max) / 2.0],
            "half_extents_world": [half_x, half_y, half_z],
            "rot_world": identity_rot,
            "label": f"{label_prefix}_{label}",
        }

    bars = [
        _bar(wall_y_min, wall_y_max, hole_z_max, wall_z_max, "top"),
        _bar(wall_y_min, wall_y_max, wall_z_min, hole_z_min, "bottom"),
        _bar(wall_y_min, hole_y_min, hole_z_min, hole_z_max, "left"),
        _bar(hole_y_max, wall_y_max, hole_z_min, hole_z_max, "right"),
    ]
    return [bar for bar in bars if bar is not None]


def build_cylinder_obstacle_capsule(
    *,
    center_world: Sequence[float],
    radius_m: float,
    height_m: float,
    label: str = "cylinder",
) -> dict[str, Any]:
    """Build a vertical cylindrical obstacle as a single capsule (a line
    segment plus radius), centered at ``center_world`` and spanning
    ``height_m`` along world Z.

    A capsule is the natural, exact proxy for a cylinder here -- same
    reasoning as the arm's own elongated links (see
    ``elesim_controller.robot.arm.planning.collision``'s module docstring):
    it needs no new distance math, since ``capsule_capsule_gap``/
    ``capsule_box_gap`` already handle a capsule vs. any arm-link shape. The
    only inexactness is the capsule's hemispherical caps rounding off the
    cylinder's flat top/bottom -- a real cylinder is very slightly wider at
    its flat edges than the capsule that spans the same axis at the same
    radius, so this is a conservative (not permissive) proxy.

    Returns a dict in the exact shape
    ``elesim_controller.robot.arm.planning.collision.CollisionModel``'s
    ``obstacle_capsules`` expects (``p0_world``/``p1_world``/``radius``/
    ``label``), ready to drop into a collision-model JSON's
    ``"obstacle_capsules"`` list.
    """
    cx, cy, cz = (float(x) for x in center_world)
    half_h = float(height_m) / 2.0
    return {
        "p0_world": [cx, cy, cz - half_h],
        "p1_world": [cx, cy, cz + half_h],
        "radius": float(radius_m),
        "label": str(label),
    }


def build_collision_model(
    *,
    blueprint_path: Path,
    source_root: Path,
    output: Path,
    default_radius: float = 0.03,
    go2_body_length_m: float | None = None,
    go2_body_width_m: float | None = None,
    go2_body_height_m: float | None = None,
    go2_urdf_path: Path | None = None,
    # Unused by the auto-detect path below (build_go2_body_shapes always
    # walks the real robot structure from "base"); kept only for CLI
    # backward compatibility (`--go2-urdf-link`) and because
    # `parse_urdf_link_collision_box` -- the single-named-link reader this
    # used to drive -- is still a standalone, tested utility.
    go2_urdf_link_name: str = DEFAULT_GO2_URDF_LINK_NAME,
    go2_ignore_links: Sequence[str] = DEFAULT_GO2_IGNORE_LINKS,
    self_ignore_links: Sequence[str] = DEFAULT_SELF_IGNORE_LINKS,
    ignore_pairs: Sequence[tuple[str, str]] = DEFAULT_IGNORE_PAIRS,
    config_path: Path | None = None,
    discovery_samples: int = 500,
    obstacle_boxes: Sequence[Mapping[str, Any]] = (),
    obstacle_capsules: Sequence[Mapping[str, Any]] = (),
) -> Path:
    with open(blueprint_path, "r", encoding="utf-8") as handle:
        blueprint = json.load(handle)

    context = _build_context(config_path=config_path, source_root=source_root) if config_path is not None else None
    fk_joint_chain = context["fk_joint_chain"] if context is not None else None
    link_capsules = build_link_capsules(
        blueprint=blueprint, source_root=source_root, fk_joint_chain=fk_joint_chain, box_fit_names=BOX_FIT_LINK_NAMES
    )
    link_boxes = build_link_boxes(blueprint=blueprint, source_root=source_root, box_fit_names=BOX_FIT_LINK_NAMES)

    go2_body_shapes: dict[str, Any] | None = None
    if go2_body_length_m is None and go2_body_width_m is None and go2_body_height_m is None:
        resolved_urdf_path = go2_urdf_path if go2_urdf_path is not None else source_root / "assets" / "go2" / "go2.urdf"
        if resolved_urdf_path.is_file():
            go2_body_shapes = build_go2_body_shapes(resolved_urdf_path)

    resolved_ignore_pairs = frozenset(frozenset(pair) for pair in ignore_pairs)
    if context is not None:
        discovered = _discover_extra_ignore_pairs(
            context=context,
            link_capsules=link_capsules,
            link_boxes=link_boxes,
            default_radius=default_radius,
            ignore_pairs=resolved_ignore_pairs,
            self_ignore_links=frozenset(self_ignore_links),
            discovery_samples=discovery_samples,
        )
        newly_found = discovered - resolved_ignore_pairs
        if newly_found:
            print(
                f"discovered {len(newly_found)} always-colliding pair(s) to ignore: "
                f"{sorted(tuple(sorted(p)) for p in newly_found)}"
            )
        resolved_ignore_pairs = resolved_ignore_pairs | discovered
    # Sorted for deterministic artifact output -- resolved_ignore_pairs is a
    # set of frozensets, whose iteration order is not guaranteed stable.
    ignore_pairs = sorted(tuple(sorted(pair)) for pair in resolved_ignore_pairs)

    go2_capsules: list[dict[str, Any]] = []
    go2_boxes: list[dict[str, Any]] = []
    go2_leg_segments: list[dict[str, Any]] = []
    if go2_body_shapes is not None:
        go2_capsules = go2_body_shapes["go2_capsules"]
        go2_boxes = go2_body_shapes["go2_boxes"]
        go2_leg_segments = go2_body_shapes["go2_leg_segments"]
    elif go2_body_length_m is not None:
        half_extents = [float(go2_body_length_m) / 2.0, float(go2_body_width_m or 0.0) / 2.0, float(go2_body_height_m or 0.0) / 2.0]
        if half_extents[1] <= 0.0 or half_extents[2] <= 0.0:
            raise ValueError(
                "go2_body_width_m and go2_body_height_m must both be positive when go2_body_length_m is set"
            )
        go2_boxes.append(
            {
                "center_body": [0.0, 0.0, 0.0],
                "half_extents_body": half_extents,
                "rot_body": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
                "label": "go2_chassis",
            }
        )

    data = {
        "schema_version": 1,
        "link_capsules": link_capsules,
        "link_boxes": link_boxes,
        "default_radius": float(default_radius),
        "go2_capsules": go2_capsules,
        "go2_boxes": go2_boxes,
        "go2_leg_segments": go2_leg_segments,
        "obstacle_boxes": [dict(box) for box in obstacle_boxes],
        "obstacle_capsules": [dict(capsule) for capsule in obstacle_capsules],
        "ignore_pairs": [list(pair) for pair in ignore_pairs],
        "go2_ignore_links": list(go2_ignore_links),
        "self_ignore_links": list(self_ignore_links),
    }

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(data, ensure_ascii=True, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    if not go2_capsules and not go2_boxes:
        print(
            "warning: no GO2 body dimensions supplied; go2_collision_check will report every "
            "configuration clear of the GO2 body until --go2-body-length/--go2-body-width/"
            "--go2-body-height are provided"
        )
    return output


__all__ = [
    "bounding_box",
    "bounding_box_excluding_outlier_cluster",
    "bounding_box_pair_by_largest_gap",
    "bounding_capsule",
    "build_collision_model",
    "build_cylinder_obstacle_capsule",
    "build_go2_body_shapes",
    "build_link_boxes",
    "build_link_capsules",
    "build_wall_with_hole_boxes",
    "chain_axis_capsule",
    "fit_plate_link_boxes",
    "parse_urdf_link_collision_box",
]
