"""Self-contained reader for the ``collision_model.json`` artifact, for visualization only.

Deliberately does not depend on ``elesim_controller`` (this package is
independently deployable and shares only ``elesim_protocol`` with the rest
of the monorepo -- see the top-level architecture docs) -- just re-parses
the same JSON schema that
``elesim_controller.robot.arm.planning.collision.CollisionModel`` and
``misc/tooling/model_builder`` produce/consume. No distance-math or
collision-checking logic here, only enough to draw each link's proxy
capsule(s)/box(es) at its live world pose for a debug overlay -- the real
collision checking still happens controller-side.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field, replace
from typing import Any, Mapping, Optional

import numpy as np


@dataclass(frozen=True)
class LinkCapsuleGeom:
    p0_local: np.ndarray
    p1_local: np.ndarray
    radius: float


@dataclass(frozen=True)
class LinkBoxGeom:
    center_local: np.ndarray
    half_extents_local: np.ndarray


@dataclass(frozen=True)
class Go2CapsuleGeom:
    p0_body: np.ndarray
    p1_body: np.ndarray
    radius: float
    label: str = ""


@dataclass(frozen=True)
class Go2BoxGeom:
    center_body: np.ndarray
    half_extents_body: np.ndarray
    rot_body: np.ndarray
    label: str = ""


@dataclass(frozen=True)
class WorldBoxGeom:
    """A static obstacle box already in world coordinates (e.g. a wall) --
    unlike ``Go2BoxGeom``, no ``go2_pos``/``go2_rot`` transform is needed to
    place it, since it never moves with GO2's base."""

    center_world: np.ndarray
    half_extents_world: np.ndarray
    rot_world: np.ndarray
    label: str = ""


@dataclass(frozen=True)
class WorldCapsuleGeom:
    """A static obstacle capsule already in world coordinates (e.g. a
    cylindrical post) -- mirrors ``elesim_controller...collision.WorldCapsule``
    for visualization purposes only."""

    p0_world: np.ndarray
    p1_world: np.ndarray
    radius: float
    label: str = ""


@dataclass(frozen=True)
class Go2LegSegmentGeom:
    """One leg segment (hip/thigh/calf)'s static geometry -- see
    ``elesim_model_builder.collision_model.build_go2_body_shapes``'s
    docstring for the ``leg_q`` ordering convention this assumes."""

    name: str
    parent: str
    origin_from_parent: np.ndarray
    origin_rot_in_parent: np.ndarray
    axis_in_parent: np.ndarray
    leg_q_index: int
    shape_type: str  # "capsule" | "box"
    p0_local: np.ndarray = field(default_factory=lambda: np.zeros(3))
    p1_local: np.ndarray = field(default_factory=lambda: np.zeros(3))
    radius: float = 0.0
    center_local: np.ndarray = field(default_factory=lambda: np.zeros(3))
    half_extents_local: np.ndarray = field(default_factory=lambda: np.zeros(3))
    rot_local: np.ndarray = field(default_factory=lambda: np.eye(3))


@dataclass(frozen=True)
class CollisionGeometryModel:
    link_capsules: Mapping[str, LinkCapsuleGeom] = field(default_factory=dict)
    link_boxes: Mapping[str, tuple[LinkBoxGeom, ...]] = field(default_factory=dict)
    go2_capsules: tuple[Go2CapsuleGeom, ...] = ()
    go2_boxes: tuple[Go2BoxGeom, ...] = ()
    go2_leg_segments: tuple[Go2LegSegmentGeom, ...] = ()
    obstacle_boxes: tuple[WorldBoxGeom, ...] = ()
    obstacle_capsules: tuple[WorldCapsuleGeom, ...] = ()
    self_ignore_links: frozenset = field(default_factory=frozenset)
    go2_ignore_links: frozenset = field(default_factory=frozenset)

    def is_inert(self, link_name: str) -> bool:
        """A link excluded from *both* the self- and GO2-body checks is never
        actually used by collision checking at all -- its own fitted geometry
        can be arbitrarily bad (e.g. ``plate``'s, deliberately unfit and
        excluded rather than chased further -- see
        ``elesim_model_builder.collision_model``'s module docstring) without
        it mattering functionally, but drawing it in this debug overlay would
        show that same bad, never-checked geometry as if it meant something.
        """
        return link_name in self.self_ignore_links and link_name in self.go2_ignore_links

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "CollisionGeometryModel":
        link_capsules = {
            str(name): LinkCapsuleGeom(
                p0_local=np.asarray(entry["p0_local"], dtype=float).reshape(3),
                p1_local=np.asarray(entry["p1_local"], dtype=float).reshape(3),
                radius=float(entry["radius"]),
            )
            for name, entry in dict(data.get("link_capsules", {}) or {}).items()
        }
        link_boxes = {
            str(name): tuple(
                LinkBoxGeom(
                    center_local=np.asarray(box["center_local"], dtype=float).reshape(3),
                    half_extents_local=np.asarray(box["half_extents_local"], dtype=float).reshape(3),
                )
                for box in entries
            )
            for name, entries in dict(data.get("link_boxes", {}) or {}).items()
        }
        go2_capsules = tuple(
            Go2CapsuleGeom(
                p0_body=np.asarray(entry["p0_body"], dtype=float).reshape(3),
                p1_body=np.asarray(entry["p1_body"], dtype=float).reshape(3),
                radius=float(entry["radius"]),
                label=str(entry.get("label", "")),
            )
            for entry in data.get("go2_capsules", []) or []
        )
        go2_boxes = tuple(
            Go2BoxGeom(
                center_body=np.asarray(entry["center_body"], dtype=float).reshape(3),
                half_extents_body=np.asarray(entry["half_extents_body"], dtype=float).reshape(3),
                rot_body=np.asarray(entry["rot_body"], dtype=float).reshape(3, 3),
                label=str(entry.get("label", "")),
            )
            for entry in data.get("go2_boxes", []) or []
        )
        go2_leg_segments = tuple(
            Go2LegSegmentGeom(
                name=str(entry["name"]),
                parent=str(entry["parent"]),
                origin_from_parent=np.asarray(entry["origin_from_parent"], dtype=float).reshape(3),
                origin_rot_in_parent=np.asarray(entry["origin_rot_in_parent"], dtype=float).reshape(3, 3),
                axis_in_parent=np.asarray(entry["axis_in_parent"], dtype=float).reshape(3),
                leg_q_index=int(entry["leg_q_index"]),
                shape_type=str(entry["shape_type"]),
                p0_local=np.asarray(entry.get("p0_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3),
                p1_local=np.asarray(entry.get("p1_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3),
                radius=float(entry.get("radius", 0.0)),
                center_local=np.asarray(entry.get("center_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3),
                half_extents_local=np.asarray(entry.get("half_extents_local", [0.0, 0.0, 0.0]), dtype=float).reshape(3),
                rot_local=np.asarray(entry.get("rot_local", np.eye(3).tolist()), dtype=float).reshape(3, 3),
            )
            for entry in data.get("go2_leg_segments", []) or []
        )
        obstacle_boxes = tuple(
            WorldBoxGeom(
                center_world=np.asarray(entry["center_world"], dtype=float).reshape(3),
                half_extents_world=np.asarray(entry["half_extents_world"], dtype=float).reshape(3),
                rot_world=np.asarray(
                    entry.get("rot_world", np.eye(3).tolist()), dtype=float
                ).reshape(3, 3),
                label=str(entry.get("label", "")),
            )
            for entry in data.get("obstacle_boxes", []) or []
        )
        obstacle_capsules = tuple(
            WorldCapsuleGeom(
                p0_world=np.asarray(entry["p0_world"], dtype=float).reshape(3),
                p1_world=np.asarray(entry["p1_world"], dtype=float).reshape(3),
                radius=float(entry["radius"]),
                label=str(entry.get("label", "")),
            )
            for entry in data.get("obstacle_capsules", []) or []
        )
        self_ignore_links = frozenset(str(x) for x in data.get("self_ignore_links", []) or [])
        go2_ignore_links = frozenset(str(x) for x in data.get("go2_ignore_links", []) or [])
        return cls(
            link_capsules=link_capsules,
            link_boxes=link_boxes,
            go2_capsules=go2_capsules,
            go2_boxes=go2_boxes,
            go2_leg_segments=go2_leg_segments,
            obstacle_boxes=obstacle_boxes,
            obstacle_capsules=obstacle_capsules,
            self_ignore_links=self_ignore_links,
            go2_ignore_links=go2_ignore_links,
        )

    @classmethod
    def from_json(cls, path: str) -> "CollisionGeometryModel":
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        return cls.from_dict(data)


def world_capsule(pos: np.ndarray, rot: np.ndarray, capsule: LinkCapsuleGeom) -> tuple[np.ndarray, np.ndarray, float]:
    """Map a capsule's local endpoints into world coordinates given its link's world pose."""
    pos_arr = np.asarray(pos, dtype=float).reshape(3)
    rot_arr = np.asarray(rot, dtype=float).reshape(3, 3)
    p0_world = pos_arr + rot_arr @ capsule.p0_local
    p1_world = pos_arr + rot_arr @ capsule.p1_local
    return p0_world, p1_world, capsule.radius


# Pairs of corner indices (see world_box_corners' sign ordering) that form
# the box's 12 edges: two corners are connected iff they differ along
# exactly one axis (i.e. their index differs in exactly one bit, since the
# corner order is sx-major/sy-mid/sz-minor -> index = 4*sx_idx+2*sy_idx+sz_idx).
BOX_EDGE_INDICES = (
    [(i, i ^ 1) for i in range(8) if i < (i ^ 1)]
    + [(i, i ^ 2) for i in range(8) if i < (i ^ 2)]
    + [(i, i ^ 4) for i in range(8) if i < (i ^ 4)]
)


def world_box_corners(pos: np.ndarray, rot: np.ndarray, box: LinkBoxGeom) -> np.ndarray:
    """The box's 8 corners in world space, keeping its real orientation.

    Deliberately not reduced to a world-axis-aligned bounds pair: a link far
    out on the chain (``gripper_base``/claws) can accumulate a lot of
    rotation from upstream bend/roll joints, and re-enclosing a rotated box
    in a world AABB inflates it back out towards its bounding sphere's size
    -- confirmed live, a badly oversized and misoriented box relative to
    the actual mesh. Draw as a 12-edge wireframe (``BOX_EDGE_INDICES``)
    using individual lines instead, which can represent any orientation.
    """
    pos_arr = np.asarray(pos, dtype=float).reshape(3)
    rot_arr = np.asarray(rot, dtype=float).reshape(3, 3)
    center_world = pos_arr + rot_arr @ box.center_local
    signs = np.array([[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])
    corners_local = signs * box.half_extents_local.reshape(1, 3)
    return center_world.reshape(1, 3) + corners_local @ rot_arr.T


def go2_leg_world_shapes(
    go2_pos: np.ndarray, go2_rot: np.ndarray, leg_q: np.ndarray, segments: tuple[Go2LegSegmentGeom, ...]
) -> list[tuple[str, str, Any]]:
    """FK each leg segment to its live world pose, mirroring
    ``elesim_controller.robot.arm.planning.collision.go2_leg_world_shapes`` for
    visualization purposes (this module deliberately doesn't import that
    package -- see the module docstring). Returns a list of
    ``(name, shape_type, shape_data)`` where ``shape_data`` is
    ``(p0, p1, radius)`` for a capsule or ``(center, half_extents, rot)`` for
    a box, all in world coordinates.
    """
    go2_pos_arr = np.asarray(go2_pos, dtype=float).reshape(3)
    go2_rot_arr = np.asarray(go2_rot, dtype=float).reshape(3, 3)
    leg_q_arr = np.asarray(leg_q, dtype=float).reshape(-1)
    world_pose: dict[str, tuple[np.ndarray, np.ndarray]] = {"base": (go2_pos_arr, go2_rot_arr)}

    results: list[tuple[str, str, Any]] = []
    for segment in segments:
        parent_pos, parent_rot = world_pose[segment.parent]
        joint_angle = float(leg_q_arr[segment.leg_q_index])
        axis = segment.axis_in_parent
        norm = float(np.linalg.norm(axis))
        if norm > 1e-12:
            axis_unit = axis / norm
        else:
            axis_unit = axis
        cos_a, sin_a = np.cos(joint_angle), np.sin(joint_angle)
        # Rodrigues' rotation formula -- avoids a scipy dependency here since
        # this module is otherwise numpy-only.
        k = axis_unit
        kx = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
        joint_rot = np.eye(3) + sin_a * kx + (1.0 - cos_a) * (kx @ kx)
        seg_pos = parent_pos + parent_rot @ segment.origin_from_parent
        seg_rot = parent_rot @ segment.origin_rot_in_parent @ joint_rot
        world_pose[segment.name] = (seg_pos, seg_rot)

        if segment.shape_type == "box":
            center = seg_pos + seg_rot @ segment.center_local
            box_rot = seg_rot @ segment.rot_local
            results.append((segment.name, "box", (center, segment.half_extents_local, box_rot)))
        else:
            p0 = seg_pos + seg_rot @ segment.p0_local
            p1 = seg_pos + seg_rot @ segment.p1_local
            results.append((segment.name, "capsule", (p0, p1, segment.radius)))
    return results


def _box_aabb_extent(half_extents: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """World-axis-aligned half-extent of an oriented box -- mirrors
    ``elesim_controller...collision._box_aabb_extent``."""
    return np.abs(np.asarray(rot, dtype=float).reshape(3, 3)) @ np.asarray(half_extents, dtype=float).reshape(3)


def simplify_go2_to_bounding_box(
    model: CollisionGeometryModel, *, leg_q: Optional[np.ndarray] = None
) -> CollisionGeometryModel:
    """Mirrors ``elesim_controller...collision.simplify_go2_to_bounding_box``
    (the controller-side optimization that plans arm paths against a single
    merged GO2 box instead of ~15 individual torso/head/leg shapes) purely
    for visualization -- so this debug overlay shows what path planning is
    *actually* checking against, not stale per-shape detail that no longer
    reflects the real check. Same body-frame-AABB tradeoff: conservative,
    since the merged box can cover real empty space around/between
    individual leg shapes.

    Returns ``model`` unchanged if it has no GO2 shapes to merge.
    """
    lo = np.full(3, np.inf)
    hi = np.full(3, -np.inf)

    def _extend_box(center: np.ndarray, half_extents: np.ndarray, rot: np.ndarray) -> None:
        nonlocal lo, hi
        center_arr = np.asarray(center, dtype=float).reshape(3)
        extent = _box_aabb_extent(half_extents, rot)
        lo = np.minimum(lo, center_arr - extent)
        hi = np.maximum(hi, center_arr + extent)

    def _extend_capsule(p0: np.ndarray, p1: np.ndarray, radius: float) -> None:
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
        for _name, shape_type, shape_data in go2_leg_world_shapes(
            np.zeros(3), np.eye(3), leg_q, model.go2_leg_segments
        ):
            if shape_type == "box":
                center, half_extents, rot = shape_data
                _extend_box(center, half_extents, rot)
            else:
                p0, p1, radius = shape_data
                _extend_capsule(p0, p1, radius)

    if not np.all(np.isfinite(lo)):
        return model  # nothing to merge

    center = (lo + hi) / 2.0
    half_extents = (hi - lo) / 2.0
    merged_box = Go2BoxGeom(
        center_body=center, half_extents_body=half_extents, rot_body=np.eye(3), label="go2_merged"
    )
    return replace(model, go2_boxes=(merged_box,), go2_capsules=(), go2_leg_segments=())


def resolve_collision_geometry_model_path() -> str:
    """Resolve the collision-model artifact path for a debug-overlay visualization.

    Mirrors ``elesim_controller.robot.arm.planning.collision.load_collision_model_for_config``'s
    ``ELESIM_COLLISION_MODEL`` env override without importing that package.
    Falls back to this repo's conventional location (``controller/config/
    collision_model.json``), resolved from this file's own path rather than
    any simulator config path -- the two configs don't share a directory
    depth (simulator's default config is ``config/default.yaml``, one level
    under the repo root; the controller's is two levels under it).
    """
    override = os.environ.get("ELESIM_COLLISION_MODEL", "").strip()
    if override:
        return override
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    return os.path.join(repo_root, "controller", "config", "collision_model.json")


def load_collision_geometry_model() -> Optional[CollisionGeometryModel]:
    """Best-effort load; ``None`` if the artifact hasn't been generated (visualization is simply unavailable)."""
    model_path = resolve_collision_geometry_model_path()
    if not os.path.isfile(model_path):
        return None
    try:
        return CollisionGeometryModel.from_json(model_path)
    except Exception:
        return None


__all__ = [
    "BOX_EDGE_INDICES",
    "CollisionGeometryModel",
    "Go2BoxGeom",
    "Go2CapsuleGeom",
    "Go2LegSegmentGeom",
    "LinkBoxGeom",
    "LinkCapsuleGeom",
    "WorldBoxGeom",
    "WorldCapsuleGeom",
    "go2_leg_world_shapes",
    "load_collision_geometry_model",
    "resolve_collision_geometry_model_path",
    "simplify_go2_to_bounding_box",
    "world_box_corners",
    "world_capsule",
]
