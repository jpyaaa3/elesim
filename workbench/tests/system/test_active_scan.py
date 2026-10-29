"""Actual FK/view planner against an analytic sensor fixture, without Genesis/ROS.

Only this evaluator and its private sensor fixture know the sphere. The scan
consumer receives RGB-D, source identity, measured q and the nominal arm model.
This is a kinematic software test, not physics/rendering acceptance evidence.
"""
from pathlib import Path
import json

import numpy as np
import pytest

from elesim_pilot.reconstruction import Measurement, Pipeline, Task
from elesim_pilot.reconstruction.scan import ActiveScan
from elesim_pilot.reconstruction.scan_collision import ScanCollision
from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
from elesim_pilot.vision.multiview import ViewPlanner
from elesim_pilot.vision.partial_cloud import ArmCameraModel, SensorObservation
from elesim_pilot.vision.perception.depth_pose import CameraIntrinsics
from elesim_protocol.scene_artifact import SceneArtifact


ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('distance_m,can_move', [(0.3, False), (0.4, True)])
def test_real_view_planner_acquires_multiple_sensor_only_silhouettes(tmp_path, distance_m, can_move):
    base = np.eye(4)
    base[2, 3] = .8
    model_dir = ROOT/'payload/data/models/planning/zed-mini'
    model = ArmCameraModel(ROOT/'payload/data/models/arm/default.json',
        ROOT/'payload/data/calibration/cameras/zed_mini.hand_eye.json', base_world_transform=base)
    initial = np.array([-.132063, .382158, -.33934, .558898])
    initial_pose = model.camera_pose(initial)
    center = initial_pose[:3, 3]+initial_pose[:3, :3]@np.array([0., 0., distance_m])
    radius = .045
    intr = CameraIntrinsics(80., 80., 39.5, 29.5, 80, 60)
    row, col = np.mgrid[:60, :80]
    directions = np.stack([(col-intr.cx)/intr.fx, (row-intr.cy)/intr.fy, np.ones((60,80))], axis=-1)
    acquired = []

    def capture(q, certificate):
        assert certificate['accepted']
        pose = model.camera_pose(q)
        world_rays = directions@pose[:3, :3].T
        offset = pose[:3, 3]-center
        a = np.sum(world_rays**2, axis=-1)
        b = 2*np.sum(world_rays*offset, axis=-1)
        c = float(offset@offset-radius**2)
        discriminant = b*b-4*a*c
        hit = discriminant > 0
        depth = np.zeros((60,80), np.float32)
        depth[hit] = ((-b[hit]-np.sqrt(discriminant[hit]))/(2*a[hit])).astype(np.float32)
        hit &= depth > .1
        image = np.zeros((60,80,3), np.uint8)
        image[hit, 2] = 255
        acquired.append(q.copy())
        return Measurement(SensorObservation(image, depth, 1., intr, len(acquired),
            100.+len(acquired), 'sim-fixture', 'boot', 'calibration', tuple(q)), base_world_transform=base)

    collision = ScanCollision(ArmSelfCollision(model_dir, joint_uncertainty=[.003,.04,.04,.04]), base)
    planner = ViewPlanner(model, collision_checker=collision, candidates=1200)
    pipeline = Pipeline.from_config(ROOT/'payload/config/pilot/scan.example.yaml')
    result, report = ActiveScan(pipeline, model, planner, max_views=3).run(
        capture, initial, Task('sphere-scan', 'target'), tmp_path/'scan')
    if can_move:
        assert len(acquired) >= 2, report
        assert not np.allclose(acquired[0], acquired[1])
        assert report['accepted_views'] >= 2
    else:
        # The nearer object intersects the arm's tracking-error envelope.
        assert len(acquired) == 1
        assert report['stop_reason'] == 'no_safe_distinct_view'
    points = result.objects[0].shape.cloud.observed
    radial_error = np.abs(np.linalg.norm(points-center, axis=1)-radius)
    assert np.percentile(radial_error, 95) < .005
    assert report['geometry_complete_claimed'] is False
    assert 0 < report['coverage']['fraction'] < 1
    assert report['coverage']['unobserved_directions']
    print(json.dumps({'evidence': 'analytic kinematic sensor fixture; no Genesis/ROS',
        'distance_m': distance_m, 'views': len(acquired), 'accepted_views': report['accepted_views'],
        'surface_p95_mm': float(np.percentile(radial_error, 95)*1000),
        'view_direction_proxy': report['coverage']['fraction'], 'stop_reason': report['stop_reason']}))
    artifact = SceneArtifact.load(report['scene_manifest'])
    assert artifact.objects[0].observed_points == len(points)
    assert artifact.objects[0].predicted_points == 0
    for path in (tmp_path/'scan/frames').glob('*.npz'):
        SensorObservation.load(path)  # Strict packet excludes truth/mesh fields.
