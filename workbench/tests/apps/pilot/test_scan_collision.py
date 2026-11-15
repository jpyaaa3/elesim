from types import SimpleNamespace as NS
from pathlib import Path
import itertools

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


def test_view_arrival_reserves_room_for_the_next_normal_certificate():
    from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
    root=Path(__file__).resolve().parents[4]/'payload/data/models/planning/zed-mini'
    uncertainty=np.array([.001,.015,.015,.015])
    arm=ArmSelfCollision(root,joint_uncertainty=uncertainty)
    normal=ScanCollision(arm,np.eye(4))
    reserve=ScanCollision(arm,np.eye(4),reserve_tracking=True)
    # Exact cached goal and measured arrival from installed run-29. Only
    # 21.7% of the allowed error was enough to leave the reusable safe set.
    target=np.array([-.15923274993748118,.21265288828715612,-.13868058463209948,.581926593530298])
    measured=np.array([-.1591443568468094,.21238024532794952,-.13543402254581452,.5818884134292602])
    assert np.all(abs(target-measured)<uncertainty)
    assert normal.configuration_free(target)
    assert not normal.configuration_free(measured)
    assert not reserve.configuration_free(target)
    interior=np.array([-.08,.4,-.2,.4])
    assert reserve.configuration_free(interior)
    for signs in itertools.product((-1,1),repeat=4):
        assert normal.configuration_free(interior+uncertainty*signs)


def test_view_arrival_also_reserves_tracking_room_from_observed_surfaces():
    def poses(q):
        pose=np.eye(4);pose[0,3]=q[0];return {'tip':pose}
    arm=NS(names=['tip'],radii={'tip':np.array([.01])},local={'tip':np.zeros((1,3))},
        ancestry={'tip':[0]},bounds=np.array([[-1.,1.]]*4),uncertainty=np.array([.003,0.,0.,0.]),
        joints=[dict(index=0,multiplier=1.,kind='prismatic',t=np.eye(4))],link_poses=poses,
        pair_lipschitz=np.zeros((1,4)),required_gap=np.zeros(1),pair_clearances=lambda q:np.ones(1),
        configuration_free=lambda q:True)
    normal=ScanCollision(arm,np.eye(4));reserve=ScanCollision(arm,np.eye(4),reserve_tracking=True)
    normal.tree=reserve.tree=cKDTree([[0.,0.,0.]])
    edge=np.array([.0295,0.,0.,0.])
    assert normal.configuration_free(edge) and not reserve.configuration_free(edge)
    interior=np.array([.035,0.,0.,0.])
    assert reserve.configuration_free(interior)
    assert normal.configuration_free(interior-[.003,0.,0.,0.])
