from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as Rot

from elesim_sim.robot.go2.locomotion.types import ALL_LEGS, LegId
from elesim_sim.simulation.genesis.utils import to_numpy_1d_copy as _to_numpy_1d

GO2_STAND_Q: Dict[str, float] = {
    "FL_hip_joint": 0.1,
    "FL_thigh_joint": 0.8,
    "FL_calf_joint": -1.5,
    "FR_hip_joint": -0.1,
    "FR_thigh_joint": 0.8,
    "FR_calf_joint": -1.5,
    "RL_hip_joint": 0.1,
    "RL_thigh_joint": 1.0,
    "RL_calf_joint": -1.5,
    "RR_hip_joint": -0.1,
    "RR_thigh_joint": 1.0,
    "RR_calf_joint": -1.5,
}

# Crouched pose aligned with go2-convex-mpc Pin init (lower COM for walking).
GO2_READY_Q: Dict[str, float] = {
    "FL_hip_joint": 0.1,
    "FL_thigh_joint": 0.9,
    "FL_calf_joint": -1.8,
    "FR_hip_joint": -0.1,
    "FR_thigh_joint": 0.9,
    "FR_calf_joint": -1.8,
    "RL_hip_joint": 0.1,
    "RL_thigh_joint": 0.9,
    "RL_calf_joint": -1.8,
    "RR_hip_joint": -0.1,
    "RR_thigh_joint": 0.9,
    "RR_calf_joint": -1.8,
}

GO2_LEG_JOINTS: Tuple[str, ...] = tuple(GO2_STAND_Q.keys())

HIP_OFFSET_BODY: Dict[LegId, np.ndarray] = {
    LegId.FL: np.array([0.1934, 0.0465, 0.0], dtype=float),
    LegId.FR: np.array([0.1934, -0.0465, 0.0], dtype=float),
    LegId.RL: np.array([-0.1934, 0.0465, 0.0], dtype=float),
    LegId.RR: np.array([-0.1934, -0.0465, 0.0], dtype=float),
}

TROT_PHASE_OFFSET: Dict[LegId, float] = {
    LegId.FL: 0.0,
    LegId.RR: 0.0,
    LegId.FR: 0.5,
    LegId.RL: 0.5,
}

# Genesis GO2 URDF merges fixed foot joints into calf; foot tip is below calf origin.
FOOT_LOCAL_OFFSET_IN_CALF = np.array([0.0, 0.0, -0.213], dtype=float)


def _leg_joint_names(leg: LegId) -> Tuple[str, str, str]:
    prefix = leg.value
    return (
        f"{prefix}_hip_joint",
        f"{prefix}_thigh_joint",
        f"{prefix}_calf_joint",
    )


@dataclass
class Go2KinematicsModel:
    leg_dof_idx: Dict[LegId, List[int]]
    all_leg_dof_idx: List[int]
    stand_q: np.ndarray
    ready_q: np.ndarray
    foot_link_names: Dict[LegId, str]
    hip_link_names: Dict[LegId, str]

    @classmethod
    def from_entity(cls, entity) -> "Go2KinematicsModel":
        leg_dof_idx: Dict[LegId, List[int]] = {}
        stand_vals: List[float] = []
        ready_vals: List[float] = []
        all_leg_dof_idx: List[int] = []

        for leg in ALL_LEGS:
            idxs: List[int] = []
            for joint_name in _leg_joint_names(leg):
                try:
                    joint = entity.get_joint(joint_name)
                except (KeyError, ValueError, AttributeError) as exc:
                    raise ValueError(f"GO2 model is missing required joint {joint_name!r}") from exc
                raw_idxs = joint.dofs_idx_local
                if raw_idxs is None:
                    raise ValueError(f"GO2 joint {joint_name!r} has no local DOF index")
                joint_idxs = np.asarray(raw_idxs, dtype=int).reshape(-1)
                if joint_idxs.size != 1:
                    raise ValueError(
                        f"GO2 joint {joint_name!r} must own exactly one DOF, got {joint_idxs.size}"
                    )
                for idx in joint_idxs:
                    idx_int = int(idx)
                    idxs.append(idx_int)
                    all_leg_dof_idx.append(idx_int)
                    stand_vals.append(float(GO2_STAND_Q.get(joint_name, 0.0)))
                    ready_vals.append(float(GO2_READY_Q.get(joint_name, 0.0)))
            leg_dof_idx[leg] = idxs

        if len(all_leg_dof_idx) != len(GO2_LEG_JOINTS) or len(set(all_leg_dof_idx)) != len(
            GO2_LEG_JOINTS
        ):
            raise ValueError(
                "GO2 model must expose 12 distinct leg DOFs in FL/FR/RL/RR order; "
                f"got {all_leg_dof_idx}"
            )

        foot_link_names = {leg: f"{leg.value}_calf" for leg in ALL_LEGS}
        hip_link_names = {leg: f"{leg.value}_hip" for leg in ALL_LEGS}

        return cls(
            leg_dof_idx=leg_dof_idx,
            all_leg_dof_idx=all_leg_dof_idx,
            stand_q=np.asarray(stand_vals, dtype=float),
            ready_q=np.asarray(ready_vals, dtype=float),
            foot_link_names=foot_link_names,
            hip_link_names=hip_link_names,
        )

    def foot_link(self, entity, leg: LegId):
        return entity.get_link(self.foot_link_names[leg])

    def hip_link(self, entity, leg: LegId):
        return entity.get_link(self.hip_link_names[leg])

    def nominal_foot_body(self, leg: LegId, *, body_height_m: float) -> np.ndarray:
        hip = HIP_OFFSET_BODY[leg]
        return np.array([hip[0], hip[1], -float(body_height_m)], dtype=float)

    def read_link_pos_world(self, link) -> np.ndarray:
        return _to_numpy_1d(link.get_pos())[:3]

    def foot_local_offset_in_calf(self) -> np.ndarray:
        return FOOT_LOCAL_OFFSET_IN_CALF.copy()

    def read_foot_pos_world(self, entity, leg: LegId) -> np.ndarray:
        calf = self.foot_link(entity, leg)
        pos = _to_numpy_1d(calf.get_pos())[:3]
        quat = _to_numpy_1d(calf.get_quat())[:4]
        rot = Rot.from_quat([float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])])
        return pos + rot.apply(FOOT_LOCAL_OFFSET_IN_CALF)
