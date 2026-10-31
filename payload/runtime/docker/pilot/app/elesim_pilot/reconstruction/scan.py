"""Active static-object acquisition using model slots and measured capture callbacks.

This module has no DDS, simulator or actuator imports. The acquisition adapter
must execute the requested certified joint segment, settle, and return a fresh
source-fenced Measurement. Hidden surface completeness is never inferred from
scan saturation or a frame budget.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import cv2
from scipy.spatial import cKDTree

from elesim_pilot.vision.multiview import ScanMap, refine_joint_scan, transform
from elesim_pilot.vision.partial_cloud import VoxelCloud, reconstruct_view
from .artifacts import export_result
from .contracts import Measurement, ObjectCloud
from .pipeline import ObservationPending, PipelineCancelled


def save_observation(sensor, path):
    np.savez_compressed(path, schema_version=1, color_bgr=sensor.color_bgr,
        depth_raw=sensor.depth_raw, depth_scale=sensor.depth_scale,
        intrinsics=[sensor.intrinsics.fx, sensor.intrinsics.fy, sensor.intrinsics.cx,
                    sensor.intrinsics.cy, sensor.intrinsics.width, sensor.intrinsics.height],
        seq=sensor.seq, ts=sensor.ts, source_id=sensor.source_id,
        source_boot_id=sensor.source_boot_id, calibration_id=sensor.calibration_id,
        encoder_q=sensor.encoder_q)


class ActiveScan:
    def __init__(self, pipeline, camera_model, planner, *, max_views=12):
        if type(max_views) is not int or not 2 <= max_views <= 24:
            raise ValueError('scan view budget must be 2..24')
        self.pipeline, self.camera_model, self.planner = pipeline, camera_model, planner
        self.max_views = max_views

    def run(self, acquire, start_q, task, output, *, cancelled=lambda: False, progress=lambda event: None):
        root = Path(output)
        root.mkdir(parents=True, exist_ok=False)
        (root/'frames').mkdir()
        (root/'silhouettes').mkdir()
        (root/'calibration').mkdir()
        report = {'schema_version': 1, 'phase': 'acquiring', 'scene_id': task.scene_id,
                  'target_id': task.target_id, 'geometry_complete_claimed': False, 'views': []}
        try:
            result = self._run(acquire, start_q, task, root, report, cancelled, progress)
            report['phase'] = 'completed'
            return result, report
        except BaseException as exc:
            report['phase'] = 'cancelled' if isinstance(exc, (PipelineCancelled, InterruptedError)) else 'failed'
            report['error'] = str(exc)[:512]
            raise
        finally:
            (root/'scan.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')

    def _run(self, acquire, start_q, task, root, report, cancelled, progress):
        mapper = ScanMap(max_views=self.max_views)
        current = np.array(start_q, dtype=float)
        attempted, accepted_poses, captures, labels = [], [], [], {}
        incremental_poses = {}
        identity, imu_calibration, last_seq, last_ts = None, None, -1, 0.
        fused, stalls = np.empty((0, 3)), 0
        fit = None
        stop = 'view_budget'
        nominal_fit = True
        callbacks = dict(cancelled=cancelled, progress=progress)
        for index in range(self.max_views):
            if cancelled():
                raise PipelineCancelled('scan cancelled')
            certificate = self.planner.collision_checker.certify_segment(
                np.asarray(start_q) if index == 0 else attempted[-1], current)
            if not certificate['accepted']:
                raise ValueError('unsafe scan segment: '+certificate['reason'])
            progress({'stage': 'moving', 'view': index, 'views': self.max_views})
            for calibration_attempt in range(8):
                if cancelled():
                    raise PipelineCancelled('scan cancelled during calibration')
                measurement = acquire(current.copy(), certificate)
                if not isinstance(measurement, Measurement):
                    raise TypeError('scan acquisition must return Measurement')
                measurement = measurement.validated()
                sensor = measurement.sensor
                source = (sensor.source_id, sensor.source_boot_id, sensor.calibration_id)
                if identity is not None and identity != source:
                    raise ValueError('source boot/calibration changed during scan')
                if sensor.seq <= last_seq or sensor.ts < last_ts:
                    raise ValueError('replayed or reordered scan frame')
                if not np.allclose(sensor.encoder_q, current, rtol=0., atol=[.001, .015, .015, .015]):
                    raise ValueError('capture does not match settled scan posture')
                if measurement.imu is not None:
                    if imu_calibration is not None and imu_calibration != measurement.imu.calibration_id:
                        raise ValueError('IMU calibration changed during scan')
                    imu_calibration = measurement.imu.calibration_id
                identity, last_seq, last_ts = source, sensor.seq, sensor.ts
                try:
                    measurement, pose, instances = self.pipeline.observe(measurement, task, **callbacks)
                    break
                except ObservationPending as error:
                    name = f'view-{index:03d}-{calibration_attempt:02d}.npz'
                    save_observation(sensor, root/'calibration'/name)
                    report.setdefault('calibration_observations', []).append(dict(view=index,
                        seq=sensor.seq, file=name, reason=str(error), diagnostics=self.pipeline.diagnostics(),
                        base_world_transform=(None if measurement.base_world_transform is None
                                              else measurement.base_world_transform.tolist())))
                    progress({'stage': 'calibrating', 'view': index, 'reason': str(error)})
                    if calibration_attempt == 7:
                        raise RuntimeError('scan calibration unavailable after 8 coherent captures: '+str(error)) from error
            observe_scene = getattr(self.planner.collision_checker, 'observe', None)
            if observe_scene is not None:
                observe_scene(measurement, pose.world_from_camera)
            nominal_fit &= np.allclose(pose.world_from_camera,
                self.camera_model.camera_pose(sensor.encoder_q), atol=1e-7)
            points = {}
            target_mask = None
            for instance in instances:
                if labels.get(instance.object_id, instance.label) != instance.label:
                    raise ValueError('object identity changed label across views')
                labels[instance.object_id] = instance.label
                if len(labels) > 16:
                    raise ValueError('scan object budget exceeded')
                partial = reconstruct_view(sensor, instance.mask, pose.world_from_camera,
                    max_points=min(20_000, 100_000//max(1, len(instances))))
                points[instance.object_id] = partial.points_camera
                if instance.object_id == task.target_id:
                    target_mask = instance.mask
            if target_mask is None or len(points[task.target_id]) < 60:
                raise ValueError('selected instance is not visible with sufficient depth; re-aim or rescan')
            # Each acquisition is recorded before any registration/model completion.
            save_observation(sensor, root/'frames'/f'{index:03d}.npz')
            np.savez_compressed(root/'silhouettes'/f'{index:03d}.npz', mask=target_mask,
                seq=sensor.seq, estimated_world_from_camera=pose.world_from_camera,
                base_world_transform=(np.empty((0, 4)) if measurement.base_world_transform is None
                                      else measurement.base_world_transform),
                observed_camera=points[task.target_id])
            if not cv2.imwrite(str(root/'silhouettes'/f'{index:03d}.png'), target_mask.astype(np.uint8)*255):
                raise OSError('could not save silhouette image')
            result = mapper.add(points[task.target_id], pose.world_from_camera,
                identity=source, seq=sensor.seq)
            row = {'view': index, 'seq': sensor.seq, 'q': list(sensor.encoder_q),
                   'accepted': bool(result['accepted']), 'reason': result['reason'],
                   'observed_points': len(points[task.target_id]), 'path_certificate': certificate,
                   'base_world_transform': (None if measurement.base_world_transform is None
                                            else measurement.base_world_transform.tolist()),
                   'pose_corrections': self.pipeline.diagnostics()['pose_corrections']}
            report['views'].append(row)
            attempted.append(current.copy())
            captures.append((sensor.encoder_q, points, index))
            previous = fused
            if result['accepted']:
                incremental_poses[index] = mapper.reference_pose@result['camera_in_reference']
            accepted_poses = list(incremental_poses.values())
            fused = mapper.points()
            if not len(fused):
                raise ValueError('first scan observation rejected')
            if len(captures) >= 4 and nominal_fit:
                progress({'stage': 'registering', 'view': index})
                candidate = refine_joint_scan(self.camera_model,
                    [(q, p[task.target_id]) for q, p, _ in captures])
                if candidate['accepted']:
                    fit = candidate
            if not nominal_fit:
                fit = None
            if fit is not None:
                fused = fit['points']
                accepted_poses = [mapper.reference_pose@fit['camera_in_reference'][i]
                                  for i in fit['used_frames']]
            if len(previous):
                fresh = float(np.mean(cKDTree(previous).query(fused)[0] > .006))
                extent = float(np.max(np.abs(np.percentile(fused, [2,98], axis=0)
                                           -np.percentile(previous, [2,98], axis=0))))
                stalls = stalls+1 if fresh < .06 and extent < .003 else 0
                row.update(new_geometry_fraction=fresh, extent_change_m=extent)
            placed = transform(fused, mapper.reference_pose)
            report['coverage'] = self.planner.coverage(placed, accepted_poses)
            report['accepted_views'] = len(accepted_poses)
            report['observed_points'] = len(fused)
            progress({'stage': 'observed', 'view': index, 'accepted': row['accepted'],
                'accepted_views': len(accepted_poses), 'observed_points': len(fused),
                'coverage': report['coverage']})
            if index == self.max_views-1:
                break
            progress({'stage': 'choosing_view', 'view': index})
            choice = self.planner.choose(placed, sensor.encoder_q, accepted_poses, attempted,
                cancelled=cancelled)
            if choice is None:
                stop = 'no_safe_distinct_view'
                break
            if len(accepted_poses) >= 4 and stalls >= 2 and choice['expected_visibility_gain'] < .015:
                stop = 'information_saturated'
                break
            row['next_expected_visibility_gain'] = float(choice['expected_visibility_gain'])
            current = choice['q']
        if cancelled():
            raise PipelineCancelled('scan cancelled')
        # Rebuild all scene objects using one set of accepted camera transforms.
        clouds = {}
        if fit is not None:
            used = set(fit['used_frames'])
            poses = {i: mapper.reference_pose@fit['camera_in_reference'][i] for i in used}
        else:
            poses = incremental_poses
        for i, (_, by_object, _) in enumerate(captures):
            if i not in poses:
                continue
            for key, values in by_object.items():
                if len(values):
                    clouds.setdefault(key, VoxelCloud(voxel_m=self.pipeline.voxel_m)).add(transform(values, poses[i]))
        final_clouds = [ObjectCloud(key, labels[key], task.role(key), task.frame_id, cloud.points())
                        for key, cloud in clouds.items()]
        target_points = next(cloud.observed for cloud in final_clouds if cloud.object_id == task.target_id)
        report['accepted_views'] = len(poses)
        report['observed_points'] = len(target_points)
        report['coverage'] = self.planner.coverage(target_points, list(poses.values()))
        for i, row in enumerate(report['views']):
            row['fusion_used'] = i in poses
        report.update(stop_reason=stop, needs_more_observation=True,
            world_placement_independently_verified=False,
            joint_fit=None if fit is None else {k:v for k,v in fit.items()
                if k not in {'points', 'camera_in_reference'}})
        progress({'stage': 'reconstructing', 'stop_reason': stop})
        result = self.pipeline.finish(final_clouds, task, sensor, views=len(captures),
            extra_provenance={'scan': {'stop_reason': stop, 'geometry_complete_claimed': False,
                'coverage': report['coverage'], 'reference': 'first accepted estimated camera frame'},
                'imu_calibration_id': imu_calibration}, **callbacks)
        report['predicted_points'] = sum(len(obj.shape.predicted) for obj in result.objects)
        report['model_execution'] = result.provenance['model_execution']
        if cancelled():
            raise PipelineCancelled('scan cancelled')
        manifest = export_result(result, root/'scene')
        if cancelled():
            raise PipelineCancelled('scan cancelled during export')
        report['scene_manifest'] = str(manifest)
        return result
