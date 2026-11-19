"""Inspect candidate view gates using a saved first RGB-D frame, without motion.

This replay never certifies base paths or issues commands. Optionally it checks
stationary arm segments against the recorded scene. Its body pose comes from
the nearest saved UI telemetry, so it is diagnostic, not a live acceptance test.
"""
from __future__ import annotations

import argparse
import inspect
import json
from pathlib import Path
from types import SimpleNamespace as NS
import time

import numpy as np
from scipy.spatial import cKDTree

from elesim_pilot.mobile_manipulation.base_planner import body_frame
from elesim_pilot.mobile_manipulation.contracts import Settings, angle
from elesim_pilot.mobile_manipulation.local_map import rectangular_footprint
from elesim_pilot.mobile_manipulation.see_planner import MobileSeePlanner
from elesim_pilot.reconstruction.scan_collision import ScanCollision
from elesim_pilot.robot.arm.self_collision import ArmSelfCollision
from elesim_pilot.vision.multiview import ScanMap, transform
from elesim_pilot.vision.partial_cloud import ArmCameraModel, SensorObservation
from elesim_pilot.vision.see import SeeBackend, SeeOptions


class GeometryProbe(MobileSeePlanner):
    def __init__(self, *args, **kwargs):
        self.check_stationary = kwargs.pop('check_stationary', False)
        self.candidate_gates = []
        self.not_certified = 0
        self.checked = []
        super().__init__(*args, **kwargs)

    def camera_at(self, pose, q, snapshot):
        camera = super().camera_at(pose, q, snapshot)
        caller = inspect.currentframe().f_back
        # Only the ranking call; skip the many calls inside the IK residual.
        if caller.f_code.co_name == '_choose_mobile':
            values = caller.f_locals
            direction = camera[:3, 3] - values['center']
            direction /= max(np.linalg.norm(direction), 1e-9)
            novelty = float(np.degrees(np.arccos(np.clip(values['seen'] @ direction, -1., 1.))).min())
            extent = self._contains_extent(values['observed'], camera)
            self.candidate_gates.append(dict(novelty_deg=novelty, contains_extent=extent,
                base_pose=np.asarray(pose).tolist(), q=np.asarray(q).tolist()))
        return camera

    def motion(self, base_pose, q, snapshot, *, cancelled):
        self.not_certified += 1
        stationary = np.linalg.norm(base_pose-snapshot['pose']) < 1e-8
        if self.check_stationary and stationary:
            result = super().motion(base_pose, q, snapshot, cancelled=cancelled)
            self.checked.append(dict(stationary=True,accepted=result is not None,
                base_pose=base_pose.tolist(),q=q.tolist()))
            return result
        self.checked.append(dict(stationary=bool(stationary),accepted=False,
            base_pose=base_pose.tolist(),q=q.tolist()))
        return None  # This replay does not have the original complete floor map.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scan', type=Path, required=True)
    parser.add_argument('--telemetry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--data', type=Path, default=Path('payload/data'))
    parser.add_argument('--intermediate', action='store_true',
        help='diagnose intermediate poses; never authorize motion')
    parser.add_argument('--check-stationary', action='store_true',
        help='check recorded-scene arm segments; no base path is certified')
    args = parser.parse_args()
    report = json.loads((args.scan/'scan.json').read_text())
    row = report['views'][0]
    sensor = SensorObservation.load(args.scan/'frames/000.npz')
    with np.load(args.scan/'silhouettes/000.npz', allow_pickle=False) as raw:
        points, camera, base = (raw[k].copy() for k in ('observed_camera','estimated_world_from_camera','base_world_transform'))
    samples = [json.loads(line) for line in args.telemetry.open()]
    host = min(samples, key=lambda item: abs(item['wall'] - row['timestamp_s']))['host']
    pos, rpy = np.asarray(host['go2_base_pos']), np.asarray(host['go2_base_rpy'])
    base_pose = np.r_[pos[:2], rpy[2]]
    body = body_frame(base_pose, pos[2], rpy[:2])
    arm_in_body = np.linalg.inv(body) @ base
    model = ArmCameraModel(args.data/'models/arm/default.json',
        args.data/'calibration/cameras/zed_mini.hand_eye.json', base_world_transform=base)
    arm = ArmSelfCollision(args.data/'models/planning/zed-mini', joint_uncertainty=[.001,.015,.015,.015])
    collision = ScanCollision(arm, base, reserve_tracking=True)
    collision.observe(NS(sensor=sensor, base_world_transform=base), camera)
    home = np.asarray(json.loads((args.data/'policies/hug-d100/manifest.json').read_text())['home'])
    snapshot = dict(joint=dict(q=np.asarray(sensor.encoder_q)), pose=base_pose,
        base=dict(pos=pos, rpy=rpy))
    mount = NS(arm_base_world_transform=lambda p,r: body_frame(np.r_[p[:2],r[2]],p[2],r[:2]) @ arm_in_body)
    ports = NS(arm=arm, collision=collision, arm_in_body=arm_in_body,
        model=NS(meta={'home':home}), service=NS(_go2_arm_mount=mount), s=Settings(),
        footprint=rectangular_footprint(((-.38,.38),(-.25,.25))))
    mobility = NS(attempted=[])
    planner = GeometryProbe(model, ports, mobility, roi_height_m=Settings().scan_roi_height_m,
        backend=SeeBackend(options=SeeOptions(density_per_m3=5_000_000)),
        check_stationary=args.check_stationary)
    mapper = ScanMap()
    result = mapper.add(points, camera, identity=(sensor.source_id,sensor.source_boot_id,sensor.calibration_id), seq=sensor.seq)
    if not result['accepted']:
        raise RuntimeError('first observation not accepted')
    observed = transform(mapper.points(), mapper.reference_pose)
    planner.set_observations([(points, camera)], sensor.intrinsics)
    started = time.monotonic()
    proposals = planner.frontiers(observed, cancelled=lambda: False)
    choice = None
    if proposals is not None:
        planner.update_base(snapshot)
        choice, _ = planner._choose_mobile(proposals, observed, snapshot, [camera], cancelled=lambda: False)
    if proposals is not None and args.check_stationary:
        blockers = cKDTree(np.concatenate([observed, collision.cloud.points()]))
        for candidate in planner.candidate_gates:
            if (np.linalg.norm(np.asarray(candidate['base_pose'])-base_pose)<1e-8 and
                    8<=candidate['novelty_deg']<=42 and candidate['contains_extent']):
                pose=planner.camera_at(base_pose,np.asarray(candidate['q']),snapshot)
                visible=planner._visible(proposals[:,6:9],pose,blockers)
                toward=proposals[:,6:9]-pose[:3,3]
                toward/=np.maximum(np.linalg.norm(toward,axis=1)[:,None],1e-9)
                visible &= np.sum(toward*proposals[:,3:6],axis=1)>.15
                candidate['visible_frontiers']=int(visible.sum())
                if visible.any():
                    plan=MobileSeePlanner.motion(planner,base_pose,np.asarray(candidate['q']),
                        snapshot,cancelled=lambda:False)
                    candidate['recorded_scene_arm_segment_accepted']=plan is not None
    intermediate = []
    if proposals is not None and args.intermediate:
        center = np.mean(np.percentile(observed, [5, 95], axis=0), axis=0)
        seen = camera[:3,3]-center; seen /= np.linalg.norm(seen)
        blockers = cKDTree(np.concatenate([observed, collision.cloud.points()]))
        start_q = np.asarray(sensor.encoder_q)
        for index, candidate in enumerate(planner.candidate_gates):
            delta = np.asarray(candidate['base_pose'])-base_pose; delta[2] = angle(delta[2])
            target_q = np.asarray(candidate['q'])
            for fraction in (.25, .5, .75):
                target_base = base_pose+fraction*delta
                target = start_q+fraction*(target_q-start_q)
                pose = planner.camera_at(target_base, target, snapshot)
                direction = pose[:3,3]-center; direction /= np.linalg.norm(direction)
                novelty = float(np.degrees(np.arccos(np.clip(seen@direction,-1.,1.))))
                extent = planner._contains_extent(observed, pose)
                visible = planner._visible(proposals[:,6:9], pose, blockers)
                toward=proposals[:,6:9]-pose[:3,3]
                toward/=np.maximum(np.linalg.norm(toward,axis=1)[:,None],1e-9)
                visible &= np.sum(toward*proposals[:,3:6],axis=1)>.15
                intermediate.append(dict(candidate=index,fraction=fraction,novelty_deg=novelty,
                    contains_extent=extent,visible_frontiers=int(visible.sum()),
                    base_pose=target_base.tolist(),q=target.tolist()))
        if args.check_stationary:
            for candidate in intermediate:
                if (np.linalg.norm(np.asarray(candidate['base_pose'])-base_pose)<1e-8 and
                        8<=candidate['novelty_deg']<=42 and candidate['contains_extent'] and
                        candidate['visible_frontiers']):
                    plan = MobileSeePlanner.motion(planner, base_pose,
                        np.asarray(candidate['q']), snapshot, cancelled=lambda: False)
                    candidate['recorded_scene_arm_segment_accepted'] = plan is not None
    output = dict(scope='offline first-frame geometry gates; no motion or floor-path certification',
        recorded_search=row.get('view_search'), elapsed_s=time.monotonic()-started,
        replay_search=planner.last_search, candidates=planner.candidate_gates,
        candidates_reaching_certification=planner.not_certified, checked=planner.checked, intermediate=intermediate,
        selected=None if choice is None else dict(q=choice['q'].tolist(),
            base_pose=choice['base_pose'].tolist(),policy=choice['policy'],
            visible_frontiers=choice['visible_frontiers'],certificate=choice['path_certificate']))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(output,stream,indent=2,allow_nan=False);stream.write('\n')
    print(json.dumps({k:v for k,v in output.items() if k not in ('candidates','intermediate')},indent=2))


if __name__ == '__main__':
    main()
