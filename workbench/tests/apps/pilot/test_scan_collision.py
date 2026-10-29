from types import SimpleNamespace as NS

import numpy as np
from scipy.spatial import cKDTree

from elesim_pilot.reconstruction.scan_collision import ScanCollision


def test_observed_obstacle_between_safe_endpoints_rejects_the_whole_segment():
    def poses(q):
        pose = np.eye(4)
        pose[0, 3] = q[0]
        return {'tip': pose}
    arm = NS(names=['tip'], radii={'tip': np.array([.01])}, local={'tip': np.zeros((1,3))},
        ancestry={'tip': [0]}, bounds=np.array([[-1.,1.]]*4), uncertainty=np.array([.003,.04,.04,.04]),
        joints=[dict(index=0, multiplier=1., kind='prismatic', t=np.eye(4))], link_poses=poses,
        certify_segment=lambda *a: {'accepted': True, 'reason': 'test-self-free'},
        configuration_free=lambda q: True)
    safety = ScanCollision(arm, np.eye(4))
    safety.tree = cKDTree([[0., 0., 0.]])
    left, right = np.array([-.2, 0., 0., 0.]), np.array([.2, 0., 0., 0.])
    assert safety.configuration_free(left) and safety.configuration_free(right)
    result = safety.certify_segment(left, right)
    assert not result['accepted'] and result['reason'] == 'observed_surface_collision_on_path'
