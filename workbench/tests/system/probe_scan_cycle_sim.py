"""Sim-only fixture/evidence adapter around the installed, real DDS runtime.

No frames, measurements, transport, reconstruction or scene commands are mocked.
The initial red target is private ground truth; Pilot never receives this file.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import importlib.util
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evidence', type=Path, required=True)
    args, runtime_args = parser.parse_known_args()
    for sibling in ('elesim_pilot', 'elesim_ui'):
        assert importlib.util.find_spec(sibling) is None, sibling
    import cv2
    import numpy as np
    from elesim_sim.main import main as run_sim
    from elesim_sim.runtime import RuntimePrep, SimRuntime

    output = args.evidence
    output.mkdir(parents=True, exist_ok=True)
    evidence = dict(scope='installed Sim, actual physics/RGBD/DDS/WebRTC', edits=[],
        fixture_ready=False, sibling_imports_absent=True, started=time.time())

    def save():
        (output/'sim.json').write_text(json.dumps(evidence, indent=2, allow_nan=False)+'\n')

    def array(value):
        if hasattr(value, 'detach'):
            value = value.detach().cpu().numpy()
        return np.asarray(value).copy()

    original_init = RuntimePrep.init_genesis
    original_edit = SimRuntime._apply_scene_command

    def fixture(prep, *values, **kwargs):
        original_init(prep, *values, **kwargs)
        app = prep.app
        assert not app.cfg.use_hardware and not app.cfg.use_go2
        scene = app.sim_scene
        q = (-.132063, .382158, -.339340, .558898)
        scene.mover.set_4dof_instant(*q)
        app.state_source.hold_for_scene_rebuild(scene.mover.current_4dof_q())
        axes = scene.camera_axes_world(hand_eye_path=app.cfg.hand_eye_config)
        assert axes is not None, 'actual hand-eye pose unavailable'
        origin, look, _ = axes
        center = origin+.4*look/np.linalg.norm(look)
        scene.set_sim_target_position(center)
        app.spawn = replace(app.spawn, sim_target_xyz=tuple(map(float, center)))
        evidence.update(fixture_ready=True, truth=dict(center_m=center.tolist(),
            shape=app.spawn.sim_target_shape,height_m=float(app.spawn.sim_target_height),
            radius_m=float(app.spawn.sim_target_radius), initial_q=list(q),
            camera_origin_m=origin.tolist(), camera_look=look.tolist()),
            genesis_version=__import__('genesis').__version__, fixture_ready_at=time.time())
        if app.spawn.sim_target_shape == 'jar':
            import hashlib
            from elesim_sim.simulation.target_geometry import jar_mesh_path, jar_profile
            path = jar_mesh_path(app.spawn.sim_target_radius, app.spawn.sim_target_height)
            data = path.read_bytes()
            (output/'jar-target.obj').write_bytes(data)
            evidence['truth']['profile_rz_m'] = jar_profile(
                app.spawn.sim_target_radius, app.spawn.sim_target_height)
            evidence['truth']['mesh'] = dict(file='jar-target.obj', sha256=hashlib.sha256(data).hexdigest(),
                provenance='EleSim procedural jar v1; not a ShapeNet dataset sample',
                convexify=False, decimate=False)
        save()

    def edit(runtime, command):
        app, scene = runtime.app, runtime.app.sim_scene
        started = time.time()
        row = dict(command=command.command, started=started)
        spec = scene.camera_render_worker.spec
        row.update(camera_target_shape=spec.target_shape,camera_target_height_m=spec.target_height)
        assert spec.target_shape == evidence['truth']['shape']
        assert np.isclose(spec.target_height,evidence['truth']['height_m'])
        if scene.sim_target_entity is not None:
            target_pos = array(scene.sim_target_entity.get_pos())
            row['original_target_position_m'] = target_pos.tolist()
            row['original_target_hidden_before_load'] = bool(np.allclose(target_pos,[-8.,-8.,-8.],atol=1e-6))
        before = {ent.name:dict(q=array(ent.get_qpos()), v=array(ent.get_dofs_velocity()),
            position=array(ent.get_pos()), quat=array(ent.get_quat())) for ent in scene.scene.entities}
        pids = {name:proc.pid for name,proc in scene.camera_render_worker.processes.items()}
        try:
            reason = original_edit(runtime, command)
            row.update(reason=reason, ok=True, revision=app.imported_scene.revision,
                elapsed_s=time.time()-started, timings=scene.scene_rebuilder.last_timings)
            row['entity_state_preserved'] = True
            row['entity_pose_errors'] = {}
            for ent in scene.scene.entities:
                if ent.name not in before:
                    continue
                old = before[ent.name]
                for key,value in (('q',ent.get_qpos()), ('v',ent.get_dofs_velocity()),
                        ('position',ent.get_pos()), ('quat',ent.get_quat())):
                    current = array(value)
                    error = float(np.max(np.abs(current-old[key]))) if current.size else 0.
                    if error > 1e-6:
                        row['entity_state_preserved'] = False
                        row['entity_pose_errors'][ent.name+'/'+key] = error
            row['camera_processes_preserved'] = pids == {
                name:proc.pid for name,proc in scene.camera_render_worker.processes.items()}
            row['frames'] = {}
            for stream in ('observer','hand_eye_preview'):
                frame = scene.frame_hub.latest(stream)
                assert frame is not None and frame.ts >= started, (stream, frame)
                filename = f'{stream}-after-{len(evidence["edits"])}.png'
                assert cv2.imwrite(str(output/filename), frame.color_bgr)
                row['frames'][stream] = dict(ts=frame.ts, file=filename,
                    width=int(frame.color_bgr.shape[1]), height=int(frame.color_bgr.shape[0]))
            return reason
        except BaseException as exc:
            row.update(ok=False, error=str(exc), elapsed_s=time.time()-started)
            raise
        finally:
            evidence['edits'].append(row)
            save()

    save()
    sys.argv = [sys.argv[0], *runtime_args]
    with patch.object(RuntimePrep, 'init_genesis', fixture), patch.object(SimRuntime, '_apply_scene_command', edit):
        run_sim()


if __name__ == '__main__':
    main()
