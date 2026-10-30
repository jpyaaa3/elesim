"""Exercise production scene commands, controller rebinding and both cameras.

Run in the installed Sim image on a single selected GPU. DDS/WebRTC and physical
Robot are not started. A separate system smoke covers real DDS commands.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import time
from unittest.mock import patch


def fixture_bundle(folder, *, rings, segments, phase):
    folder.mkdir(parents=True, exist_ok=False)
    vertices = [(0.8, 0., .34)]
    for ring in range(1, rings):
        theta = math.pi*ring/rings
        for col in range(segments):
            phi = 2*math.pi*col/segments
            radius = .12*(1+.12*math.sin(7*theta+phase)*math.cos(5*phi))
            vertices.append((.8+radius*math.sin(theta)*math.cos(phi),
                radius*math.sin(theta)*math.sin(phi), .22+radius*math.cos(theta)))
    vertices.append((.8, 0., .10))
    faces = []
    for col in range(segments):
        nxt = (col+1) % segments
        faces.append((0, 1+col, 1+nxt))
        faces.append((len(vertices)-1, 1+(rings-2)*segments+nxt, 1+(rings-2)*segments+col))
    for ring in range(rings-2):
        for col in range(segments):
            a, b = 1+ring*segments+col, 1+ring*segments+(col+1)%segments
            faces.extend(((a, a+segments, b+segments), (a, b+segments, b)))
    mesh = (''.join('v %.9f %.9f %.9f\n'%v for v in vertices)
        + ''.join('f %d %d %d\n'%(a+1,b+1,c+1) for a,b,c in faces)).encode()
    (folder/'object.obj').write_bytes(mesh)
    manifest = {'schema_version':1, 'scene_id':folder.name, 'frame_id':'world', 'units':'m',
        'provenance':{'source':'runtime scene rebuild probe'}, 'objects':[{'object_id':'object',
        'label':'probe object', 'role':'obstacle', 'mesh_file':'object.obj',
        'sha256':hashlib.sha256(mesh).hexdigest(), 'observed_points':len(vertices), 'predicted_points':0}]}
    raw = (json.dumps(manifest, sort_keys=True)+'\n').encode()
    (folder/'scene.json').write_bytes(raw)
    return {'manifest':folder.name+'/scene.json', 'expected_revision':hashlib.sha256(raw).hexdigest(),
        'vertices':len(vertices), 'faces':len(faces), 'mesh_sha256':hashlib.sha256(mesh).hexdigest()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config')
    parser.add_argument('model_bundle')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import genesis as gs
    import numpy as np
    from elesim_protocol import SimMappingConfig, SimulationCommandRequest
    from elesim_sim.control_state import SimulationStateSource
    from elesim_sim.runtime import AssetProcessor, GenesisApp, RuntimePrep, SimRuntime, load_app_config
    from elesim_sim.simulation.operator_control import SimulationOperatorCommand, SimulationOperatorMailbox
    from elesim_sim.vision.frame_hub import FrameHub
    from workbench.research.debug.profile_sim_walking import wait_for_camera_frames, advance_controlled_step

    output = args.output.absolute()
    output.parent.mkdir(parents=True, exist_ok=True)
    root = output.parent/'imports'
    cases = [fixture_bundle(root/'new-a', rings=9, segments=18, phase=.47),
             fixture_bundle(root/'new-b', rings=101, segments=102, phase=1.13)]
    result = {'complete':False, 'scope':'production runtime scene commands, physical state, controller identity, first frames; no DDS/WebRTC', 'cases':cases, 'runs':[]}
    def save(): output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    save()
    bundle = load_app_config(args.config, mode='remote')
    assert not bundle.sim_config.use_hardware
    cfg = replace(bundle.sim_config, build_dir=str(Path(args.model_bundle).resolve()),
        enable_viewer=False, genesis_performance_mode=False, genesis_warm_nonconvex=True,
        sim_camera_enable=True, sim_observer_camera_enable=True,
        scene_manifest='', scene_import_root=str(root))
    hub = FrameHub(('rgbd', 'hand_eye_preview', 'observer'))
    mailbox = SimulationOperatorMailbox()
    mailbox.set_session('ui-probe', 'scene-probe')
    state = SimulationStateSource(bundle.mapping_config)
    app = GenesisApp(params=bundle.sim_param, cfg=cfg, limit=bundle.joint_limit,
        model=replace(bundle.spawn_config, sim_target_enable=True),
        urdf_export_cfg=bundle.urdf_export_config, go2_locomotion_config=bundle.go2_locomotion_config,
        mapping_cfg=bundle.mapping_config, frame_hub=hub, state_source=state, operator_mailbox=mailbox)
    try:
        began = time.monotonic()
        urdf = AssetProcessor(app).prepare_assets()
        prep = RuntimePrep(app)
        with patch('elesim_sim.vision.sim_camera.SimCameraPublisher', return_value=None):
            prep.init_genesis(urdf, attach_scene_cameras=False)
        scene = app.sim_scene
        scene.configure_camera_render_worker(prep.camera_render_spec(urdf), {
            'hand_eye_preview':(cfg.sim_camera_width,cfg.sim_camera_height),
            'observer':(cfg.sim_observer_camera_width,cfg.sim_observer_camera_height)},
            timeout_s=cfg.camera_worker_start_timeout_s, wait=True)
        scene.configure_frame_dispatchers()
        scene.start_frame_dispatchers()
        wait_for_camera_frames(scene, hub, cfg)
        runtime = SimRuntime(app)
        initial_scene = id(scene.scene)
        controller = scene.go2._controller
        solver = controller._solver
        camera_pids = {name: process.pid for name, process in scene.camera_render_worker.processes.items()}
        result['startup_s'] = time.monotonic()-began
        result['nonconvex_kernel_prepared'] = bool(scene.scene.sim.rigid_solver.collider.collider_config.has_nonconvex_nonterrain)
        assert result['nonconvex_kernel_prepared']
        save()

        def run(name, arguments, *, ok=True, restored=False):
            robot = scene.go2_entity
            q, v = robot.get_qpos().cpu().numpy().copy(), robot.get_dofs_velocity().cpu().numpy().copy()
            started, wall_started = time.monotonic(), time.time()
            request = SimulationCommandRequest.from_payload({'schema_version':1, 'request_id':name+str(len(result['runs'])),
                'session_id':'scene-probe', 'command':name, 'arguments':arguments})
            mailbox.enqueue(SimulationOperatorCommand.from_request(request, ui_id='ui-probe'))
            runtime._apply_operator_commands()
            elapsed = time.monotonic()-started
            replies = mailbox.take_results()
            assert len(replies)==1 and replies[0].payload.ok is ok, replies
            assert id(scene.scene)==initial_scene
            assert scene.go2._controller is controller and controller._solver is solver
            assert {name: process.pid for name, process in scene.camera_render_worker.processes.items()} == camera_pids
            np.testing.assert_allclose(scene.go2_entity.get_qpos().cpu().numpy(), q, atol=1e-6)
            np.testing.assert_allclose(scene.go2_entity.get_dofs_velocity().cpu().numpy(), v, atol=1e-6)
            assert controller._entity is scene.go2_entity and scene.mover.entity is scene.go2_entity
            if ok or restored:
                for stream in ('hand_eye_preview','observer'):
                    assert hub.latest(stream) is not None and hub.latest(stream).ts >= wall_started
            assert bool(scene.scene.sim.rigid_solver.collider.collider_config.has_nonconvex_nonterrain)
            row = {'command':name, 'ok':ok, 'total_s':elapsed, 'reason':replies[0].payload.reason,
                'revision':app.imported_scene.revision if app.imported_scene else None,
                'paused':runtime.operator.paused, 'epoch':runtime.operator.epoch,
                'physics_state_verified':True, 'controller_and_solver_preserved':True,
                'camera_frames_fresh':ok or restored,
                'nonconvex_kernel_retained':True, 'camera_processes_preserved':True}
            if ok:
                row.update(scene.scene_rebuilder.last_timings)
            result['runs'].append(row)
            save()
            print('SCENE_RUNTIME',json.dumps(row),flush=True)

        run('load_scene', {k:cases[0][k] for k in ('manifest','expected_revision')})
        run('load_scene', {k:cases[1][k] for k in ('manifest','expected_revision')})
        from elesim_sim.simulation import runtime_scene_edit
        original_start = runtime_scene_edit._start_cameras
        failures = [True]
        def fail_once(*values):
            if failures.pop() if failures else False: raise RuntimeError('injected candidate camera failure')
            return original_start(*values)
        with patch.object(runtime_scene_edit, '_start_cameras', fail_once):
            run('load_scene', {k:cases[0][k] for k in ('manifest','expected_revision')}, ok=False, restored=True)
        assert runtime.operator.paused and app.imported_scene.revision == cases[1]['expected_revision']
        run('load_scene', {'manifest':cases[0]['manifest'], 'expected_revision':'0'*64}, ok=False)
        run('clear_scene', {})
        assert not app.imported_scene_entities
        arm = scene.mover.current_4dof_q()
        for _ in range(100): advance_controlled_step(scene, (0.,0.,0.), arm)
        pos = scene.go2_entity.get_pos().cpu().numpy()
        assert pos[2] > .16 and not controller._faulted
        result['post_rebuild_stationary_steps'] = 100
        result['complete'] = True
    finally:
        save()
        app.sim_scene.close_frame_dispatchers()
        app.sim_scene.close_camera_publishers()
        gs.destroy()


if __name__ == '__main__':
    main()
