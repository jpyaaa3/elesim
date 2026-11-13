"""Render rejected native full-planner output as a labelled kinematic diagnostic."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import numpy as np

from .v2_geometry import save, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True, help='Original full trial case/provenance')
    parser.add_argument('--planner', type=Path, required=True, help='Separate diagnostic rerun output')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--gpu', required=True)
    args = parser.parse_args()
    if os.environ.get('CUDA_VISIBLE_DEVICES') != args.gpu:
        raise ValueError('Expose only the selected GPU UUID')
    provenance = json.loads((args.case/'provenance.json').read_text())
    plan = provenance['plan']
    policy = json.loads((args.planner/'planning-policy.json').read_text())
    completion = json.loads((args.planner/'completion.json').read_text())
    query = json.loads((args.case/'query.json').read_text())
    if (provenance['planner_mode'] != 'full' or policy['authored_waypoints']
            or policy['automatic_start_offset'] or completion['planner_mode'] != 'full'):
        raise ValueError('Expected an unchanged-endpoint full-planner diagnostic')
    if policy['original_start'] != query['start'] or policy['goal'] != query['home']:
        raise ValueError('Diagnostic endpoints differ from the original trial')
    source, positions, native_dt = None, None, None
    for candidate in completion['candidates']:
        for item in candidate.get('diagnostic_only_trajectories', []):
            path = args.planner/f"candidate-{candidate['attempt']:04d}"/item['file']
            with np.load(path, allow_pickle=False) as data:
                points = data['positions'].copy()
                dt = float(data['dt_s'])
                assert bool(data['diagnostic_only']) and not bool(data['native_success'])
            if len(points) >= 2 and np.isfinite(points).all() and np.max(np.ptp(points, axis=0)) > 1e-5:
                source, positions = path, points
                native_dt = dt if np.isfinite(dt) else None
                validation = candidate['validation']
                break
        if source is not None:
            break
    if source is None:
        raise RuntimeError('No finite moving native rejected trajectory to preview')

    from workbench.research.experiments.hug_reverse_ga.prepare import digest
    from workbench.research.experiments.hug_c.configure import asset_digest
    from workbench.research.experiments.hug_c.environment import HugEnv
    from workbench.research.experiments.hug_interpolation.video import make_writer
    import torch
    from PIL import Image, ImageDraw, ImageFont

    if plan['source_sha256'] != digest() or plan['simulation_context']['model_sha256'] != asset_digest():
        raise ValueError('Physical scene source/model mismatch')
    args.output.mkdir(parents=True, exist_ok=False)
    env = HugEnv(plan, plan['cases'][0], args.output/'assets', n_envs=1,
                 backend='gpu', auto_reset=False, video=True)
    camera = env.scene.cameras['report']
    camera.set_pose(pos=(.85, -1., 1.25), lookat=(.35, .05, .55), up=(0., 0., 1.))
    env.scene.object.set_pos(torch.tensor([query['geometry']['object_position']], dtype=torch.float32, device=env.device))
    env.scene.object.set_quat(torch.tensor([query['geometry']['object_quaternion_wxyz']], dtype=torch.float32, device=env.device))
    # This deliberately renders kinematics only: no physics stepping, servo
    # execution, native timing validation or change to task acceptance.
    fps, moving_frames = 24, 192
    indices = np.r_[np.zeros(fps), np.linspace(0, len(positions)-1, moving_frames),
                    np.full(fps, len(positions)-1)]
    font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    try:
        font = ImageFont.truetype(font_path, 19)
        small = ImageFont.truetype(font_path, 15)
    except OSError:
        font = small = ImageFont.load_default()
    output = args.output/'full-rejected-kinematic.mp4'
    writer = make_writer(output, fps)
    try:
        for index in indices:
            # Uniform display progress through the stored samples. No inserted
            # pose, joint clipping, phase policy or collision repair.
            lo = int(np.floor(index)); hi = min(lo+1, len(positions)-1)
            q = positions[lo]+(positions[hi]-positions[lo])*(index-lo)
            targets = env.mapper.joint_targets(torch.as_tensor(q, dtype=torch.float32, device=env.device)[None])
            env.scene.robot.set_dofs_position(targets, dofs_idx_local=env.dofs, zero_velocity=True)
            frame = Image.fromarray(camera.render(force_render=True)[0])
            draw = ImageDraw.Draw(frame)
            draw.rectangle((0, 0, frame.width, 83), fill=(40, 7, 7))
            draw.text((13, 8), 'FULL cuRobo: REJECTED CANDIDATE | KINEMATIC PREVIEW', fill=(255, 215, 95), font=font)
            draw.text((13, 36), f'Diagnostic rerun, {args.case.name}. No physics replay. Display timing is illustrative.',
                      fill='white', font=small)
            masks = ' | '.join(f"{name}: {bool(np.asarray(value).all())}" for name, value in validation.items())
            draw.text((13, 58), masks, fill='white', font=small)
            draw.rectangle((0, frame.height-29, frame.width, frame.height), fill=(20, 20, 20))
            draw.text((12, frame.height-23),
                      f'Linear {q[0]*1000:+.1f} mm   Roll {np.rad2deg(q[1]):+.1f} deg   '
                      f'Bend totals {np.rad2deg(q[2])*5:+.1f}, {np.rad2deg(q[3])*5:+.1f} deg',
                      fill='white', font=small)
            writer.append_data(np.asarray(frame))
    finally:
        writer.close()
    save(args.output/'report.json', dict(
        status='diagnostic_only', original_case=str(args.case), planner_rerun=str(args.planner),
        trajectory=str(source), trajectory_sha256=sha(source), native_success=False,
        native_validation=validation, native_dt_s=native_dt, original_samples=len(positions),
        original_start=query['start'], home=query['home'], first_pose=positions[0].tolist(),
        last_pose=positions[-1].tolist(),
        display=dict(fps=fps, duration_s=len(indices)/fps, moving_duration_s=moving_frames/fps,
                     native_timing_preserved=False),
        physics_evaluated=False, successful_demonstration=False, training=False,
        source_note='New diagnostic rerun; rejected output was not retained by the original full trial',
        video=str(output), video_sha256=sha(output)))
    print(json.dumps(dict(video=str(output), status='diagnostic_only')), flush=True)


if __name__ == '__main__':
    main()
