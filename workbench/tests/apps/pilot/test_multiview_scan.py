"""Geometry and identity regressions for a static-object multi-view scan."""
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from elesim_pilot.vision.multiview import ScanMap, register_overlap, voxel_points, ViewPlanner, refine_joint_scan, transform
from elesim_pilot.vision.partial_cloud import ArmCameraModel
from elesim_pilot.robot.arm.self_collision import ArmSelfCollision


def test_view_candidates_respect_operating_cap_and_a_locked_joint():
    root = Path(__file__).resolve().parents[4]/'payload/data'
    model = ArmCameraModel(root/'models/arm/default.json',
        root/'calibration/cameras/zed_mini.hand_eye.json')
    bounds = np.array([[-.10, -.05], [0., 0.], [-.2, .2], [-.2, .2]])
    planner = ViewPlanner(model, collision_checker=ArmSelfCollision(root/'models/planning/zed-mini'),
        candidates=100, operating_bounds=bounds)
    assert np.all(planner.q >= bounds[:, 0]) and np.all(planner.q <= bounds[:, 1])
    assert np.all(planner.q[:, 1] == 0.)


def corner(seed=9):
    rng = np.random.default_rng(seed)
    faces = []
    for axis in range(3):
        p = rng.uniform(-.06, .06, (700, 3))
        p[:, axis] = -.06
        faces.append(p)
    return np.concatenate(faces) + [0, 0, .35]


def test_overlap_registration_recovers_a_bounded_pose_error_on_three_faces():
    target = corner()
    delta = np.eye(4)
    delta[:3, :3] = Rotation.from_euler('xyz', [1., -1.5, .7], degrees=True).as_matrix()
    delta[:3, 3] = [.004, -.003, .002]
    source = (target - delta[:3, 3]) @ delta[:3, :3]
    result = register_overlap(source, target)
    assert result['accepted']
    corrected = source @ result['transform'][:3, :3].T + result['transform'][:3, 3]
    assert np.percentile(cKDTree(target).query(corrected)[0], 95) < .001


def test_planar_overlap_does_not_claim_all_six_pose_directions_are_observed():
    x, y = np.meshgrid(np.linspace(-.08, .08, 30), np.linspace(-.08, .08, 30))
    plane = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, .3)])
    result = register_overlap(plane + [.002, 0, 0], plane)
    assert not result['refined']
    assert result['reason'] == 'underconstrained_overlap'


def test_noisy_partial_planes_are_aligned_by_surface_instead_of_sliding_point_pairs():
    rng = np.random.default_rng(37)
    target = corner()
    # The next view reveals one new face and less of an old one. Sampling
    # density changes between views, so point identities are not available.
    source = corner(38)
    source = source[(source[:, 0] < -.059) | (source[:, 2] < .291) | (np.arange(len(source)) % 5 == 0)]
    new_face = rng.uniform(-.06, .06, (700, 3))
    new_face[:, 0] = .06
    source = np.concatenate([source, new_face + [0, 0, .35]])
    source += rng.normal(0, .0009, source.shape)
    delta = np.eye(4)
    delta[:3, :3] = Rotation.from_euler('xyz', [2., -2., 1.], degrees=True).as_matrix()
    delta[:3, 3] = [.005, -.004, .003]
    displaced = (source-delta[:3, 3]) @ delta[:3, :3]
    result = register_overlap(displaced, target)
    assert result['accepted'], result
    corrected = displaced @ result['transform'][:3, :3].T + result['transform'][:3, 3]
    # Compare to the independently sampled original surfaces, including the
    # new face. Nearest-neighbour residual on the old patch alone is too weak.
    assert np.percentile(np.linalg.norm(corrected-source, axis=1), 95) < .003


def test_a_new_visible_face_is_not_counted_as_a_registration_error():
    target = corner()
    rng = np.random.default_rng(12)
    new_face = rng.uniform(-.06, .06, (1400, 3))
    new_face[:, 0] = .06
    new_face += [0, 0, .35]
    source = np.concatenate([target[::2], new_face])
    result = register_overlap(source, target)
    assert result['accepted'], result
    assert np.linalg.norm(result['transform']-np.eye(4)) < .01


def test_disjoint_views_are_rejected_and_cannot_expand_the_existing_map():
    mapper = ScanMap()
    p = corner()
    assert mapper.add(p, np.eye(4), identity=('cam', 'boot', 'cal'), seq=1)['accepted']
    before = mapper.points().copy()
    rejected = mapper.add(p + [.25, 0, 0], np.eye(4), identity=('cam', 'boot', 'cal'), seq=2)
    assert not rejected['accepted']
    np.testing.assert_array_equal(mapper.points(), before)


def test_reference_is_the_first_optical_frame_not_an_injected_world_truth():
    mapper = ScanMap()
    p = corner()
    first = np.eye(4)
    first[:3, :3] = Rotation.from_euler('z', .4).as_matrix()
    first[:3, 3] = [3, -2, .8]
    mapper.add(p, first, identity=('cam', 'boot', 'cal'), seq=1)
    np.testing.assert_allclose(mapper.points(), voxel_points(p, .003), atol=1e-12)
    assert mapper.reference_pose is not None


@pytest.mark.parametrize('identity,seq', [(('cam', 'other', 'cal'), 2), (('cam', 'boot', 'other'), 2), (('cam', 'boot', 'cal'), 1)])
def test_epochs_and_duplicate_sequences_cannot_mix_scans(identity, seq):
    mapper = ScanMap()
    p = corner()
    mapper.add(p, np.eye(4), identity=('cam', 'boot', 'cal'), seq=1)
    before = mapper.points().copy()
    result = mapper.add(p, np.eye(4), identity=identity, seq=seq)
    assert not result['accepted']
    np.testing.assert_array_equal(mapper.points(), before)


def test_repeated_views_do_not_create_new_geometry_or_unbounded_raw_storage():
    mapper = ScanMap()
    p = corner()
    original = p.copy()
    first = mapper.add(p, np.eye(4), identity=('cam', 'boot', 'cal'), seq=1)
    count = len(mapper.points())
    for seq in range(2, 8):
        result = mapper.add(p, np.eye(4), identity=('cam', 'boot', 'cal'), seq=seq)
        assert result['new_voxels'] == 0
        assert len(mapper.points()) == count
    np.testing.assert_array_equal(p, original)
    assert first['new_voxels'] == count


def test_view_planner_uses_legal_four_dof_poses_and_rejects_repeated_viewpoints():
    root = Path(__file__).resolve().parents[4] / 'payload/data'
    base = np.eye(4)
    base[2, 3] = .8
    model = ArmCameraModel(root/'models/arm/default.json', root/'calibration/cameras/zed_mini.hand_eye.json', base_world_transform=base)
    safety=ArmSelfCollision(root/'models/planning/zed-mini')
    planner = ViewPlanner(model, collision_checker=safety, candidates=3000)
    current = np.array([-.12, 0, -.1, .38])
    pose = model.camera_pose(current)
    target = np.array([.31, 0, .60])
    cloud = target + np.array([[x, y, z] for x in (-.04, .04) for y in (-.04, .04) for z in (-.04, .04)])
    choice = planner.choose(cloud, current, [pose], [current])
    assert choice is not None
    assert np.all(choice['q'] >= model.lower) and np.all(choice['q'] <= model.upper)
    assert choice['novelty_deg'] >= 8.
    assert choice['nearest_accepted_view_deg'] <= 42.
    assert safety.certify_segment(current,choice['q'])['accepted']
    assert choice['expected_visibility_gain']>0


def test_view_planner_cannot_fall_back_to_an_unchecked_path():
    root=Path(__file__).resolve().parents[4]/'payload/data'
    base=np.eye(4);base[2,3]=.8
    model=ArmCameraModel(root/'models/arm/default.json',root/'calibration/cameras/zed_mini.hand_eye.json',base_world_transform=base)
    class RejectPaths(ArmSelfCollision):
        def certify_segment(self,*args,**kwargs):return {'accepted':False,'reason':'blocked'}
    planner=ViewPlanner(model,collision_checker=RejectPaths(root/'models/planning/zed-mini'),candidates=3000)
    q=np.array([-.12,0,-.1,.38]);pose=model.camera_pose(q)
    points=np.array([.31,0,.6])+np.array([[x,y,z] for x in (-.04,.04) for y in (-.04,.04) for z in (-.04,.04)])
    assert planner.choose(points,q,[pose],[q]) is None
    assert planner.last_search['unsafe_paths']>0


def test_shared_bend_offsets_preserve_one_shape_across_different_joint_poses():
    root = Path(__file__).resolve().parents[4] / 'payload/data'
    model = ArmCameraModel(root/'models/arm/default.json', root/'calibration/cameras/zed_mini.hand_eye.json')
    qs = np.array([[-.13,.38,-.34,.56],[-.15,.1,-.2,.4],[-.05,-.3,.1,.3],
                   [-.09,.6,-.1,.28],[-.1,-.4,-.3,.5],[-.14,.2,.1,.2]])
    bias = np.array([0., 0., np.radians(.8), np.radians(-.6)])
    world = corner()+[.3, 0, .2]
    rng = np.random.default_rng(483)
    frames = []
    true_first = model.camera_pose(qs[0]+bias)
    for q in qs:
        observed = transform(world, np.linalg.inv(model.camera_pose(q+bias)))
        frames.append((q, observed+rng.normal(0, .0005, observed.shape)))
    result = refine_joint_scan(model, frames)
    assert result['accepted'], result
    np.testing.assert_allclose(result['bend_offsets_rad'], bias[2:], atol=.001)
    assert len(result['used_frames']) == len(qs)
    assert result['surface_mse_after'] < result['surface_mse_before']*.1
    rebuilt = transform(result['points'], true_first)
    assert np.percentile(cKDTree(world).query(rebuilt)[0], 95) < .002
    assert not result['absolute_world_placement_observed']


def test_repeated_joint_pose_does_not_identify_joint_zero_offsets():
    root = Path(__file__).resolve().parents[4] / 'payload/data'
    model = ArmCameraModel(root/'models/arm/default.json', root/'calibration/cameras/zed_mini.hand_eye.json')
    q = [-.13,.38,-.34,.56]
    result = refine_joint_scan(model, [(q, corner())]*4)
    assert not result['accepted']
    assert result['reason'] == 'joint_fit_unobservable_or_bound'


def test_view_preparation_services_cancellation_from_another_thread():
    import sys
    import threading
    from types import SimpleNamespace

    requested, cancelled = threading.Event(), threading.Event()
    pose = np.eye(4)
    bounds = np.column_stack((-np.ones(4),np.ones(4)))
    def camera_pose(_q):
        requested.set()
        return pose
    model = SimpleNamespace(model_digest='model',lower=bounds[:,0],upper=bounds[:,1],camera_pose=camera_pose)
    collision = SimpleNamespace(source_hashes={'arm_model.json':'model'},bounds=bounds)
    def cancel():
        requested.wait(5.)
        cancelled.set()
    thread = threading.Thread(target=cancel)
    thread.start()
    previous = sys.getswitchinterval()
    try:
        # Simulate a compute-heavy provider retaining the GIL between FK
        # calls. Control/cancel delivery needs an explicit scheduling point.
        sys.setswitchinterval(5.)
        with pytest.raises(InterruptedError,match='cancelled'):
            ViewPlanner(model,collision_checker=collision,candidates=1600,cancelled=cancelled.is_set)
    finally:
        sys.setswitchinterval(previous)
        requested.set()
        thread.join(5.)
