"""Parallel Genesis scene for wrap-grasp RL.

This builds its own scene rather than wrapping ``elesim_sim.runtime``.  The
interactive simulator is a single-environment, DDS-driven application: its
``scene.build()`` call takes no ``n_envs``, and driving one env at a time is
exactly what a parallel RL scene must avoid.  Only the shared assets are
reused -- the bundle URDF and the joint conventions -- so the existing
simulator is left untouched.

The scene holds:

* a ground plane,
* the bundle robot (GO2 quadruped **and** the continuum arm live in one URDF,
  so arm/trunk contact is a *self* contact, not an inter-entity one),
* one free-body cylinder as the graspable object.

Device and backend come from config: ``backend: gpu`` resolves to CUDA on
Linux and Metal on macOS, ``cpu`` is the portable fallback.  Nothing here
assumes NVIDIA hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

import numpy as np
import torch

from .arm_kinematics import ArmDofIndex, resolve_dof_index
from .configs.loader import WrapGraspConfig

_REPO_ROOT = Path(__file__).resolve().parents[4]
_DEFAULT_BUNDLE = "model/bundles/default"

#: GO2 leg joints, in the (hip, thigh, calf) order the held pose repeats over.
_GO2_LEG_JOINTS: tuple[str, ...] = tuple(
    f"{leg}_{part}_joint"
    for leg in ("FL", "FR", "RL", "RR")
    for part in ("hip", "thigh", "calf")
)


def resolve_bundle_dir(cfg: WrapGraspConfig) -> Path:
    """Locate the model bundle, preferring an explicit config path."""
    raw = str(cfg.scene.model_bundle).strip()
    candidate = Path(raw).expanduser() if raw else _REPO_ROOT / _DEFAULT_BUNDLE
    if not candidate.is_absolute():
        candidate = (_REPO_ROOT / candidate).resolve()
    if not candidate.is_dir():
        raise FileNotFoundError(f"model bundle directory not found: {candidate}")
    return candidate


def resolve_device(cfg: WrapGraspConfig) -> torch.device:
    """Pick the torch device for the policy side.

    ``auto`` prefers CUDA, then Apple MPS, then CPU, so the same config runs
    on a workstation GPU and on a laptop without edits.
    """
    requested = str(cfg.runtime.torch_device).strip().lower()
    if requested and requested != "auto":
        return torch.device(requested)
    if torch.cuda.is_available():
        return torch.device("cuda")
    if getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


@dataclass
class LinkIndex:
    """Link-index bookkeeping, in both index spaces Genesis uses.

    Genesis has two: ``link.idx`` is global across every entity in the scene
    (the floor is 0, the robot follows, then the support and the object), while
    ``link.idx_local`` indexes within one entity.  ``get_contacts`` reports the
    global one; ``get_links_pos`` is indexed by the local one.  Mixing them is
    an off-by-one that silently reads the neighbouring link's pose, so both are
    kept explicitly and the field names say which is which.
    """

    robot_all: tuple[int, ...]
    arm: tuple[int, ...]
    go2: tuple[int, ...]
    object_: tuple[int, ...]
    floor: tuple[int, ...]
    support: tuple[int, ...]
    segment2_mid: int
    name_by_index: dict[int, str]
    #: Robot-local indices, for get_links_pos / get_links_quat.
    arm_local: tuple[int, ...] = ()
    segment2_mid_local: int = 0

    def as_tensors(self, device: torch.device) -> dict[str, torch.Tensor]:
        def t(values: tuple[int, ...]) -> torch.Tensor:
            return torch.tensor(sorted(values), device=device, dtype=torch.int64)

        return {
            "robot": t(self.robot_all),
            "arm": t(self.arm),
            "go2": t(self.go2),
            "object": t(self.object_),
            "floor": t(self.floor),
            "support": t(self.support),
        }


class WrapGraspScene:
    """Owns the Genesis scene, its entities and the resolved index maps."""

    def __init__(
        self,
        cfg: WrapGraspConfig,
        *,
        n_envs: Optional[int] = None,
        camera_specs: Sequence[Mapping[str, Any]] = (),
    ) -> None:
        self.cfg = cfg
        # Genesis requires cameras to exist before scene.build(), so they are
        # declared up front rather than attached afterwards.
        self.camera_specs = list(camera_specs)
        self.cameras: dict[str, Any] = {}
        self.n_envs = int(n_envs if n_envs is not None else cfg.runtime.n_envs)
        self.device = resolve_device(cfg)
        self.bundle_dir = resolve_bundle_dir(cfg)
        self.scene: Any = None
        self.robot: Any = None
        self.object: Any = None
        self.floor: Any = None
        self.support: Any = None
        self.object_center: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.arm_dofs: Optional[ArmDofIndex] = None
        self.go2_leg_dofs: tuple[int, ...] = ()
        self.links: Optional[LinkIndex] = None
        self._built = False

    # -- construction ------------------------------------------------------

    @staticmethod
    def init_genesis(cfg: WrapGraspConfig) -> None:
        """Initialise Genesis once per process with the configured backend."""
        import genesis as gs

        if getattr(gs, "_initialized", False):
            return
        backend = gs.gpu if str(cfg.runtime.backend).lower() == "gpu" else gs.cpu
        gs.init(backend=backend, logging_level="warning", seed=int(cfg.runtime.seed))

    def build(self) -> "WrapGraspScene":
        import genesis as gs

        self.init_genesis(self.cfg)
        scene_cfg = self.cfg.scene
        rigid_kwargs: dict[str, Any] = {
            "max_collision_pairs": int(scene_cfg.max_collision_pairs),
        }
        self.scene = gs.Scene(
            sim_options=gs.options.SimOptions(
                dt=float(scene_cfg.dt), substeps=int(scene_cfg.solver_substeps)
            ),
            rigid_options=gs.options.RigidOptions(**rigid_kwargs),
            show_viewer=bool(scene_cfg.show_viewer),
        )

        if scene_cfg.floor:
            self.floor = self.scene.add_entity(gs.morphs.Plane())

        urdf_path = self.bundle_dir / str(scene_cfg.urdf_relpath)
        if not urdf_path.is_file():
            raise FileNotFoundError(f"bundle URDF not found: {urdf_path}")
        urdf_kwargs: dict[str, Any] = {
            "file": str(urdf_path),
            "pos": tuple(float(v) for v in scene_cfg.go2.spawn_xyz),
            "fixed": bool(scene_cfg.go2.base_fixed),
            "merge_fixed_links": False,
            "default_armature": 0.0,
        }
        if scene_cfg.decompose_robot_error_threshold is not None:
            urdf_kwargs["decompose_robot_error_threshold"] = float(
                scene_cfg.decompose_robot_error_threshold
            )
        self.robot = self.scene.add_entity(gs.morphs.URDF(**urdf_kwargs))

        support_cfg = self.cfg.support
        if support_cfg.enable:
            hx, hy = (float(v) for v in support_cfg.half_extents_xy)
            cx, cy = (float(v) for v in support_cfg.center_xy)
            top = float(support_cfg.height_m)
            self.support = self.scene.add_entity(
                gs.morphs.Box(
                    size=(hx * 2.0, hy * 2.0, top),
                    pos=(cx, cy, top * 0.5),
                    fixed=True,
                    collision=True,
                ),
                surface=gs.surfaces.Rough(color=(0.45, 0.45, 0.50, 1.0)),
            )

        obj = self.cfg.object
        if str(obj.kind).lower() != "cylinder":
            raise ValueError(f"unsupported object.kind: {obj.kind!r}")
        self.object_center = self.cfg.object_center()
        self.object = self.scene.add_entity(
            gs.morphs.Cylinder(
                radius=float(obj.radius_m),
                height=float(obj.height_m),
                pos=tuple(float(v) for v in self.object_center),
                fixed=bool(obj.fixed),
                collision=bool(obj.collision),
            ),
            surface=gs.surfaces.Rough(color=(0.85, 0.55, 0.20, 1.0)),
        )

        for spec in self.camera_specs:
            name = str(spec.get("name", f"cam{len(self.cameras)}"))
            kwargs = {k: v for k, v in spec.items() if k != "name"}
            self.cameras[name] = self.scene.add_camera(**kwargs)

        self.scene.build(n_envs=self.n_envs)
        self._built = True
        self._post_build()
        return self

    # -- post-build resolution --------------------------------------------

    def _post_build(self) -> None:
        self.arm_dofs = resolve_dof_index(self.robot, self.cfg.arm)
        self.links = self._resolve_links()
        self._configure_arm_gains()
        self._configure_go2_legs()
        self._apply_friction()

    def _resolve_links(self) -> LinkIndex:
        arm_prefixes = tuple(str(p) for p in self.cfg.arm.arm_link_prefixes)
        robot_all: list[int] = []
        arm: list[int] = []
        arm_local: list[int] = []
        go2: list[int] = []
        names: dict[int, str] = {}
        local_by_global: dict[int, int] = {}
        for link in self.robot.links:
            idx = int(link.idx)
            local = int(getattr(link, "idx_local", idx))
            name = str(link.name)
            robot_all.append(idx)
            names[idx] = name
            local_by_global[idx] = local
            if name.startswith(arm_prefixes):
                arm.append(idx)
                arm_local.append(local)
            else:
                go2.append(idx)
        obj_idx: list[int] = []
        for link in self.object.links:
            obj_idx.append(int(link.idx))
            names[int(link.idx)] = f"object/{link.name}"
        floor_idx: list[int] = []
        if self.floor is not None:
            for link in self.floor.links:
                floor_idx.append(int(link.idx))
                names[int(link.idx)] = f"floor/{link.name}"
        support_idx: list[int] = []
        if self.support is not None:
            for link in self.support.links:
                support_idx.append(int(link.idx))
                names[int(link.idx)] = f"support/{link.name}"

        mid_name = str(self.cfg.arm.segment2_mid_link)
        mid_matches = [i for i, n in names.items() if n == mid_name]
        if not mid_matches:
            raise RuntimeError(
                f"arm.segment2_mid_link={mid_name!r} not found among robot links: "
                f"{sorted(names[i] for i in arm)}"
            )
        return LinkIndex(
            robot_all=tuple(robot_all),
            arm=tuple(arm),
            go2=tuple(go2),
            object_=tuple(obj_idx),
            floor=tuple(floor_idx),
            support=tuple(support_idx),
            segment2_mid=int(mid_matches[0]),
            name_by_index=names,
            arm_local=tuple(arm_local),
            segment2_mid_local=int(local_by_global[int(mid_matches[0])]),
        )

    def _configure_arm_gains(self) -> None:
        assert self.arm_dofs is not None
        idxs = list(self.arm_dofs.all_indices)
        gains = self.cfg.arm.gains
        n = len(idxs)
        self.robot.set_dofs_kp(np.full(n, float(gains.kp)), dofs_idx_local=idxs)
        self.robot.set_dofs_kv(np.full(n, float(gains.kv)), dofs_idx_local=idxs)
        limit = float(gains.force_range)
        self.robot.set_dofs_force_range(
            np.full(n, -limit), np.full(n, limit), dofs_idx_local=idxs
        )

    def _configure_go2_legs(self) -> None:
        """Hold the quadruped legs at a fixed stance.

        The legs are scenery here, but they are still articulated DOFs; left
        uncontrolled they collapse and the trunk geometry the policy must
        avoid would drift between episodes.
        """
        go2_cfg = self.cfg.scene.go2
        if not (go2_cfg.enable and go2_cfg.freeze_legs):
            return
        dofs: list[int] = []
        for name in _GO2_LEG_JOINTS:
            try:
                joint = self.robot.get_joint(name)
            except Exception:
                continue
            raw = getattr(joint, "dofs_idx_local", None)
            if raw is None:
                continue
            dofs.extend(int(i) for i in (raw if hasattr(raw, "__iter__") else [raw]))
        if not dofs:
            return
        self.go2_leg_dofs = tuple(dofs)
        pose = np.tile(np.asarray(go2_cfg.leg_pose_rad, dtype=float), len(dofs) // 3)
        if pose.size != len(dofs):
            pose = np.resize(pose, len(dofs))
        gains = self.cfg.arm.gains
        self.robot.set_dofs_kp(np.full(len(dofs), float(gains.kp)), dofs_idx_local=dofs)
        self.robot.set_dofs_kv(np.full(len(dofs), float(gains.kv)), dofs_idx_local=dofs)
        self.robot.set_dofs_position(pose, dofs_idx_local=dofs)
        self.robot.control_dofs_position(pose, dofs_idx_local=dofs)

    def _apply_friction(self) -> None:
        friction = self.cfg.scene.friction
        if friction is None:
            return  # keep whatever the URDF and Genesis defaults provide
        for entity in (self.robot, self.object, self.floor, self.support):
            if entity is None:
                continue
            setter = getattr(entity, "set_friction", None)
            if callable(setter):
                setter(float(friction))

    # -- convenience -------------------------------------------------------

    @property
    def built(self) -> bool:
        return self._built

    def step(self) -> None:
        self.scene.step()

    def describe(self) -> dict[str, Any]:
        assert self.links is not None
        return {
            "n_envs": self.n_envs,
            "backend": str(self.cfg.runtime.backend),
            "torch_device": str(self.device),
            "dt": float(self.cfg.scene.dt),
            "max_collision_pairs": int(self.cfg.scene.max_collision_pairs),
            "bundle": str(self.bundle_dir),
            "n_robot_links": len(self.links.robot_all),
            "n_arm_links": len(self.links.arm),
            "n_go2_links": len(self.links.go2),
            "support": bool(self.support is not None),
            "object_center": [round(float(v), 4) for v in self.object_center],
            "arm_dofs": list(self.arm_dofs.all_indices) if self.arm_dofs else [],
            "go2_leg_dofs": list(self.go2_leg_dofs),
        }
