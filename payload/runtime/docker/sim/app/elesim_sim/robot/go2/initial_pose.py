"""Validate startup poses without changing Genesis URDF reference coordinates."""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from .locomotion.kinematics import GO2_STAND_Q


def validate_initial_stand_pose(entity: object) -> None:
    """Validate the pose applied by the caller after scene.build().

    Genesis interprets desc.init_qpos as qpos0, the reference subtracted by
    forward kinematics and added by set_dofs_position. Joint limits instead
    compare raw qpos. Setting qpos0 to the desired pose thus shifts the limit
    coordinates without moving the URDF reference geometry. Keep URDF zero;
    a zero-neutral calf limit warning at build time is expected. The runtime
    sets a valid stand pose before the first physics step.
    """
    validate_initial_joint_positions(entity, GO2_STAND_Q)


def validate_initial_joint_positions(entity: object, positions: Mapping[str, float]) -> None:
    """Check targets before build; never mutate qpos0 or physical joint limits."""
    if entity.is_built:
        raise RuntimeError("GO2 initial pose must be validated before scene.build()")
    for name, value in positions.items():
        joint = entity.get_joint(name)
        desc = joint.desc
        initial = np.asarray(desc.init_qpos)
        limits = np.asarray(desc.dofs_limit)
        if initial.shape != (1,) or limits.shape != (1, 2):
            raise ValueError(f"GO2 joint {name!r} must have one bounded position")
        if not np.issubdtype(initial.dtype, np.floating):
            raise ValueError(f"GO2 joint {name!r} initial position must use a floating dtype")
        if not limits[0, 0] <= value <= limits[0, 1]:
            raise ValueError(f"GO2 initial position is outside joint limits: {name}")
