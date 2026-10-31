from dataclasses import replace
import json
from types import SimpleNamespace as NS

import numpy as np
import cv2
import pytest
import yaml

from elesim_pilot.reconstruction import Measurement, Pipeline, PipelineCancelled, Task
from elesim_pilot.reconstruction.scan import ActiveScan
from elesim_pilot.vision.partial_cloud import SensorObservation
from elesim_pilot.vision.partial_cloud import VoxelCloud, reconstruct_view
from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
from elesim_protocol.scene_artifact import SceneArtifact


class CameraModel:
    def camera_pose(self, q):
        pose = np.eye(4)
        pose[0, 3] = q[0]
        return pose


class Planner:
    def __init__(self, choices=2, safe=True):
        self.choices, self.safe = choices, safe
        self.collision_checker = NS(certify_segment=lambda a, b:
            {'accepted': self.safe, 'reason': 'test-path'})

    def choose(self, points, q, poses, attempted, **kw):
        if len(attempted) > self.choices:
            return None
        return {'q': np.array(q)+[-.001, 0., 0., 0.], 'expected_visibility_gain': .2}

    def coverage(self, points, poses):
        return {'kind': 'view_direction_proxy', 'fraction': .5, 'unobserved_directions': [[0., 0., -1.]],
                'probe_count': 2, 'geometry_complete_claimed': False}


def sensor(q, seq):
    image = np.zeros((30, 60, 3), np.uint8)
    image[5:25, 5:25, 1] = 255
    image[5:25, 35:55, 2] = 255
    return SensorObservation(image, np.full((30, 60), .8, np.float32), 1.,
        CameraIntrinsics(100., 100., 30., 15., 60, 30), seq, 100.+seq,
        'sim', 'boot', 'calibration', tuple(q))


def pipeline(tmp_path):
    factory = lambda cls: {'factory': 'pilot.reconstruction_fixtures:'+cls}
    config = {'schema_version': 1, 'models': {'pose': factory('ScanPose'),
        'segmenter': factory('Segmenter'), 'completion': factory('Completion'),
        'mesher': {'factory': 'elesim_pilot.reconstruction.providers:VoxelSurface'}}}
    path = tmp_path/'models.yaml'
    path.write_text(yaml.safe_dump(config))
    return Pipeline.from_config(path)


def test_active_acquisition_records_silhouettes_and_exports_separate_scene_objects(tmp_path):
    engine = ActiveScan(pipeline(tmp_path), CameraModel(), Planner(), max_views=3)
    acquired, events = [], []
    def capture(q, certificate):
        assert certificate['accepted']
        acquired.append(q.copy())
        return Measurement(sensor(q, len(acquired)))
    result, report = engine.run(capture, np.zeros(4), Task('orchard', 'apple-1'),
        tmp_path/'scan', progress=events.append)
    assert len(acquired) == 3
    assert not np.array_equal(acquired[0], acquired[1])
    assert report['phase'] == 'completed' and report['stop_reason'] == 'view_budget'
    assert report['geometry_complete_claimed'] is False
    assert report['coverage']['fraction'] == .5
    for index in range(3):
        raw = SensorObservation.load(tmp_path/'scan/frames'/f'{index:03d}.npz')
        assert raw.seq == index+1
        with np.load(tmp_path/'scan/silhouettes'/f'{index:03d}.npz', allow_pickle=False) as data:
            assert data['mask'].dtype == bool and data['mask'].sum() == 400
            png = cv2.imread(str(tmp_path/'scan/silhouettes'/f'{index:03d}.png'), cv2.IMREAD_GRAYSCALE)
            np.testing.assert_array_equal(png > 0, data['mask'])
    artifact = SceneArtifact.load(report['scene_manifest'])
    assert [o.object_id for o in artifact.objects] == ['apple-1', 'tree-1']
    assert [o.role for o in artifact.objects] == ['target', 'obstacle']
    assert result.objects[0].shape.cloud.observed[:, 0].max() < result.objects[1].shape.cloud.observed[:, 0].min()
    assert result.objects[0].shape.predicted.shape == (1, 3)


def test_pending_calibration_collects_new_frames_before_accepting_a_view(tmp_path):
    from elesim_pilot.reconstruction.models import Model
    from elesim_pilot.reconstruction.pipeline import ObservationPending
    class Correction:
        calls = 0
        def correct(self, measurement, pose, task):
            self.calls += 1
            if self.calls < 3:
                raise ObservationPending('waiting for stable floor')
            return pose
    p = pipeline(tmp_path)
    p.corrections = (Model('pose_correction', Correction(), {'factory': 'test:Correction'}),)
    acquired = []
    base = np.eye(4)
    base[2, 3] = .8
    def capture(q, cert):
        acquired.append(q.copy())
        return Measurement(sensor(q, len(acquired)), base_world_transform=base)
    _, report = ActiveScan(p, CameraModel(), Planner(), max_views=3).run(
        capture, np.zeros(4), Task('orchard', 'apple-1'), tmp_path/'scan')
    assert len(acquired) == 5 and report['accepted_views'] == 3
    np.testing.assert_array_equal(acquired[0], acquired[2])
    assert len(report['calibration_observations']) == 2
    assert SensorObservation.load(tmp_path/'scan/frames/000.npz').seq == 3
    assert len(list((tmp_path/'scan/calibration').glob('*.npz'))) == 2
    from elesim_pilot.reconstruction.cli import load_measurements
    replay = list(load_measurements(tmp_path/'scan'))
    assert [sample.sensor.seq for sample in replay] == [1, 2, 3, 4, 5]
    for sample in replay:
        np.testing.assert_array_equal(sample.base_world_transform, base)


def test_unsupported_calibration_is_bounded_and_does_not_publish_a_scene(tmp_path):
    from elesim_pilot.reconstruction.models import Model
    from elesim_pilot.reconstruction.pipeline import ObservationPending
    class Correction:
        def correct(self, *args):
            raise ObservationPending('no supported floor')
    p = pipeline(tmp_path)
    p.corrections = (Model('pose_correction', Correction(), {'factory': 'test:Correction'}),)
    calls = []
    def capture(q, cert):
        calls.append(q)
        return Measurement(sensor(q, len(calls)))
    with pytest.raises(RuntimeError, match='calibration'):
        ActiveScan(p, CameraModel(), Planner(), max_views=3).run(
            capture, np.zeros(4), Task('orchard', 'apple-1'), tmp_path/'scan')
    assert len(calls) == 8
    assert not (tmp_path/'scan/scene/scene.json').exists()


def test_retained_joint_fit_uses_one_camera_transform_set_for_every_object(tmp_path, monkeypatch):
    poses = np.repeat(np.eye(4)[None], 4, axis=0)
    poses[:, 0, 3] = -np.arange(4)*.001
    poses[:, 1, 3] = np.arange(4)*.03
    calls = []
    def fit(model, frames):
        calls.append(len(frames))
        if len(frames) != 4:
            return {'accepted': False, 'reason': 'insufficient_new_overlap'}
        return {'accepted': True, 'used_frames': [0, 1, 2, 3], 'camera_in_reference': poses,
            'points': np.concatenate([p@t[:3,:3].T+t[:3,3] for (_, p), t in zip(frames, poses)])}
    monkeypatch.setattr('elesim_pilot.reconstruction.scan.refine_joint_scan', fit)
    sequence = []
    def capture(q, cert):
        sequence.append(q)
        return Measurement(sensor(q, len(sequence)))
    engine = ActiveScan(pipeline(tmp_path), CameraModel(), Planner(choices=4), max_views=5)
    result, report = engine.run(capture, np.zeros(4), Task('orchard', 'apple-1'), tmp_path/'scan')
    assert calls == [4, 5]
    assert report['accepted_views'] == 4
    assert [v['fusion_used'] for v in report['views']] == [True]*4+[False]
    for item, channel in zip(result.objects, [1, 2]):
        expected = VoxelCloud(voxel_m=engine.pipeline.voxel_m)
        for i, pose in enumerate(poses):
            raw = SensorObservation.load(tmp_path/'scan/frames'/f'{i:03d}.npz')
            expected.add(reconstruct_view(raw, raw.color_bgr[:, :, channel] > 0, pose).points_before)
        np.testing.assert_allclose(item.shape.cloud.observed, expected.points(), atol=1e-10)


def test_view_exhaustion_preserves_partial_result_and_reports_reason(tmp_path):
    engine = ActiveScan(pipeline(tmp_path), CameraModel(), Planner(choices=0), max_views=4)
    _, report = engine.run(lambda q, cert: Measurement(sensor(q, 1)), np.zeros(4),
        Task('s', 'apple-1'), tmp_path/'scan')
    assert report['stop_reason'] == 'no_safe_distinct_view'
    assert report['needs_more_observation'] and not report['geometry_complete_claimed']
    assert report['accepted_views'] == 1


@pytest.mark.parametrize('change,reason', [
    ({'source_boot_id': 'other'}, 'source boot/calibration'),
    ({'calibration_id': 'other'}, 'source boot/calibration'),
    ({'seq': 1}, 'replayed or reordered'),
    ({'encoder_q': (.03, 0., 0., 0.)}, 'settled scan posture'),
])
def test_mixed_or_stale_acquisition_never_publishes_scene(tmp_path, change, reason):
    calls = []
    def capture(q, cert):
        calls.append(q)
        frame = sensor(q, len(calls))
        return Measurement(replace(frame, **change) if len(calls) == 2 else frame)
    engine = ActiveScan(pipeline(tmp_path), CameraModel(), Planner(), max_views=3)
    with pytest.raises(ValueError, match=reason):
        engine.run(capture, np.zeros(4), Task('s', 'apple-1'), tmp_path/'scan')
    assert not (tmp_path/'scan/scene/scene.json').exists()
    assert json.loads((tmp_path/'scan/scan.json').read_text())['phase'] == 'failed'


def test_rejected_path_never_calls_acquisition(tmp_path):
    called = []
    engine = ActiveScan(pipeline(tmp_path), CameraModel(), Planner(safe=False))
    with pytest.raises(ValueError, match='unsafe scan segment'):
        engine.run(lambda *args: called.append(args), np.zeros(4), Task('s', 'apple-1'), tmp_path/'scan')
    assert not called


def test_cancellation_stops_next_movement_and_keeps_a_diagnostic(tmp_path):
    called = []
    def capture(q, cert):
        called.append(q)
        return Measurement(sensor(q, 1))
    engine = ActiveScan(pipeline(tmp_path), CameraModel(), Planner())
    with pytest.raises(PipelineCancelled):
        engine.run(capture, np.zeros(4), Task('s', 'apple-1'), tmp_path/'scan', cancelled=lambda: bool(called))
    assert len(called) == 1
    assert not (tmp_path/'scan/scene').exists()
    assert json.loads((tmp_path/'scan/scan.json').read_text())['phase'] == 'cancelled'
