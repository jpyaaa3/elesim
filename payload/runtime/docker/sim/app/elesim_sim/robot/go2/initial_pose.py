"""Genesis build-time GO2 posture, before neutral collision filtering."""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from .locomotion.kinematics import GO2_STAND_Q


def prepare_neutral_stand_pose(entity: object) -> None:
    """Give Genesis 1.4.1 a valid qpos0 without changing joint limits.

    URDF imports start revolute joints at zero, outside GO2 calf limits.
    Genesis' runtime set_qpos API requires an already-built scene; its joint
    descriptions own the initial positions consumed during scene.build().
    Validate every leg joint before modifying any description.
    """
    prepare_neutral_joint_positions(entity, GO2_STAND_Q)


def prepare_neutral_joint_positions(entity: object, positions: Mapping[str, float]) -> None:
    """Validate all requested scalar joint positions before updating any."""
    if entity.is_built:
        raise RuntimeError("GO2 neutral posture must be prepared before scene.build()")
    updates = []
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
            raise ValueError(f"GO2 neutral position is outside joint limits: {name}")
        position = np.asarray([value], dtype=initial.dtype)
        updates.append((desc, position))
    for desc, position in updates:
        desc.init_qpos = position
