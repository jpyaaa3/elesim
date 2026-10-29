"""Cross-role FK agreement belongs to a gate with both application roots."""
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from elesim_pilot.robot.arm.iklib.kinematics import _forward_link_tf
from elesim_pilot.robot.arm.joint_defs import JointLimit
from elesim_sim.simulation.arm_ghost import ArmGhost


ROOT = Path(__file__).resolve().parents[3]


def test_fk_matches_pilot_for_every_arm_link_and_nonidentity_base():
    context = json.loads((ROOT/'payload/data/models/arm/default.json').read_text())['context']
    context['limit'] = JointLimit(**context['limit']['value'])
    base = np.eye(4)
    base[:3, 3] = [.2, -.1, .4]
    base[:3, :3] = Rotation.from_euler('xyz', [.2, -.1, .5]).as_matrix()
    context['base_world_T'] = base
    ghost = ArmGhost(ROOT/'payload/data/models/assemblies/zed-mini/arm.urdf')
    for q in ([-.1, 0, 0, 0], [-.142, .7, .3, -.4], [0, -.7, -.2, .5]):
        actual = ghost.transforms(q, base)
        for link, (pos, rot) in _forward_link_tf(context, q).items():
            np.testing.assert_allclose(actual[link][:3, 3], pos, atol=1e-7)
            np.testing.assert_allclose(actual[link][:3, :3], rot, atol=1e-7)
