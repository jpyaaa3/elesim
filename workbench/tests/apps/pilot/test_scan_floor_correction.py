from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from elesim_pilot.reconstruction.contracts import Measurement, Pose, Task
from elesim_pilot.reconstruction.floor import FloorTilt
from elesim_pilot.reconstruction.pipeline import ObservationPending
from elesim_pilot.vision.partial_cloud import SensorObservation
from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics


def observation(seq=1, boot='boot', valid=True):
    depth = np.ones((80, 100), np.float32) if valid else np.zeros((80, 100), np.float32)
    return Measurement(SensorObservation(np.zeros((80, 100, 3), np.uint8), depth, 1.,
        CameraIntrinsics(90., 90., 50., 40., 100, 80), seq, 100.+seq*.05,
        'sim', boot, 'calibration', (0., 0., 0., 0.)))


def poses():
    actual = np.diag([1., -1., -1., 1.])
    actual[:3, 3] = [.2, -.1, 1.]
    predicted = actual.copy()
    predicted[:3, :3] = Rotation.from_euler('y', 7., degrees=True).as_matrix() @ actual[:3, :3]
    return actual, Pose('world', predicted)


def test_plane_correction_requires_stability_and_preserves_fk_origin():
    model = FloorTilt(options={'reference_normal': [0., 0., 1.]}, assets={})
    actual, pose = poses()
    task = Task('scene', 'target')
    for seq in (1, 2):
        with pytest.raises(ObservationPending):
            model.correct(observation(seq), pose, task)
    corrected = model.correct(observation(3), pose, task)
    np.testing.assert_allclose(corrected.world_from_camera, actual, atol=1e-6)
    np.testing.assert_array_equal(corrected.world_from_camera[:3, 3], pose.world_from_camera[:3, 3])
    status = model.diagnostics()
    assert status['applied'] and status['sequence'] == 3
    assert status['normal_error_deg'] == pytest.approx(7.)


def test_missing_plane_and_changed_boot_never_reuse_an_old_correction():
    model = FloorTilt(options={'reference_normal': [0., 0., 1.]}, assets={})
    _, pose = poses()
    task = Task('scene', 'target')
    for seq in (1, 2, 3):
        try:
            model.correct(observation(seq), pose, task)
        except ObservationPending:
            pass
    with pytest.raises(ObservationPending):
        model.correct(observation(4, valid=False), pose, task)
    assert not model.diagnostics()['applied']
    with pytest.raises(ObservationPending):
        model.correct(observation(1, boot='new-boot'), pose, task)
    with pytest.raises(ValueError, match='replayed'):
        model.correct(observation(1, boot='new-boot'), pose, task)


def test_reference_direction_is_an_explicit_policy():
    with pytest.raises(ValueError, match='reference_normal'):
        FloorTilt(options={}, assets={})
