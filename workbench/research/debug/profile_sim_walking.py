"""Reproduce GO2 walking in an isolated production Genesis scene, without DDS.

Run inside the installed Sim/development environment. The physical GO2+arm,
MPC and reset paths are the application's own implementations. Cameras are
disabled by default; --cameras includes the production render/dispatch workers
(not WebRTC encoding or network delivery). The fixed perception target is omitted so it cannot
obstruct the walking path. This never commands a connected Robot or live Sim.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import time
from unittest.mock import patch


def wait_for_camera_frames(scene, hub, cfg, *, clock=time.monotonic, sleep=time.sleep):
    """Account for the first EGL render before timing a short simulation trial."""
    started = clock()
    streams = ("hand_eye_preview", "observer")
    while any(hub.version(name) == 0 for name in streams):
        if scene.camera_render_worker.failure:
            raise RuntimeError("camera startup failed: " + scene.camera_render_worker.failure)
        if clock() - started >= cfg.camera_first_frame_timeout_s:
            raise TimeoutError("camera startup timed out before both first frames")
        scene.maybe_publish_camera(arm_q=None, max_hz=cfg.sim_camera_max_hz,
            sim_time_s=0., rgb_enabled=cfg.sim_camera_rgb, depth_enabled=cfg.sim_camera_depth)
        scene.maybe_publish_observer_camera(max_hz=cfg.sim_observer_camera_max_hz, sim_time_s=0.)
        sleep(.01)
    return clock() - started


def trailing_crouch_reference(sample, *, command_vx, crouch_m):
    """Diagnostic intervention: lower trailing hips, retain leading height.

    Change the body reference, never the ground height or contact evidence.
    The existing shaped forward command fades the intervention at start/stop.
    Production behavior is unchanged when the diagnostic option is omitted.
    """
    if not math.isfinite(command_vx) or not math.isfinite(crouch_m) or not 0. <= crouch_m <= .03:
        raise ValueError('diagnostic crouch needs finite velocity and 0..0.03 m drop')
    if crouch_m == 0. or command_vx == 0.:
        return sample
    signed_drop = crouch_m*max(-1., min(1., command_vx/.2))
    position = sample.desired_com_position.copy()
    rpy = sample.desired_rpy.copy()
    position[2] -= .5*abs(signed_drop)
    rpy[1] -= math.asin(signed_drop/(2.*.1934))
    return replace(sample, desired_com_position=position, desired_rpy=rpy)


def swing_reference_for_axes(original, jacobian, joint_velocity, measured_velocity,
                             desired_velocity, *, axes, scale, **options):
    """Separate vertical landing feedforward from horizontal swing reaction."""
    import numpy as np
    desired = np.asarray(desired_velocity, dtype=float).copy()
    if axes == 'vertical':
        base = np.asarray(measured_velocity)-np.asarray(jacobian)@joint_velocity
        desired[:2] = base[:2]
    elif axes != 'all':
        raise ValueError('diagnostic swing axes must be all or vertical')
    return scale*original(jacobian, joint_velocity, measured_velocity, desired, **options)


def commanded_progress_velocity(actual_velocity, commanded_velocity):
    """Retain damping of drift, vertical motion, overspeed and released intent."""
    import numpy as np
    actual = np.asarray(actual_velocity, dtype=float)
    command = np.asarray(commanded_velocity, dtype=float)
    if (actual.shape != (3,) or command.shape != (3,)
            or not np.isfinite(actual).all() or not np.isfinite(command).all()):
        raise ValueError('commanded progress needs finite three-dimensional velocities')
    speed = float(np.linalg.norm(command[:2]))
    result = np.zeros(3)
    if speed > 1e-9:
        direction = command[:2]/speed
        result[:2] = direction*np.clip(np.dot(actual[:2], direction), 0., speed)
    return result


def landing_velocity_reference(sample, *, blocked, actual_velocity, command_world, horizon_s):
    """During landing wait, request current bounded progress instead of a stop.

    This intervention never changes contact evidence, loads, liftoff scheduling
    or angular commands. It cannot accelerate toward an unachieved speed.
    """
    if not blocked:
        return sample
    velocity = commanded_progress_velocity(actual_velocity, command_world)
    position = sample.desired_com_position.copy()
    position[:2] = sample.com_position[:2]+horizon_s*velocity[:2]
    return replace(sample, desired_com_position=position, desired_com_velocity=velocity)


def has_commanded_progress(command, displacement, *, duration):
    """Reject frozen/opposite straight walking; this is not a tracking gate."""
    speed = math.hypot(command[0], command[1])
    if speed <= 1e-6 or abs(command[2]) > 1e-6:
        return True  # Straight XY trials only; turning requires a yaw metric.
    if displacement is None:
        return False
    along = (displacement[0] * command[0] + displacement[1] * command[1]) / speed
    return math.isfinite(along) and along >= max(.05, .1 * speed * duration)


def stop_cycle_results(samples, *, settle, duration, stop, cycles, sample_interval):
    """Require a settled stand at every restart boundary in a cyclic trial."""
    if stop <= 0:
        return []
    results = []
    for cycle in range(cycles):
        start = settle + cycle * (duration + stop) + duration
        end = start + stop
        rows = [row for row in samples if start <= row['t'] < end]
        last = rows[-1] if rows else None
        complete = bool(last is not None and last['t'] >= end - sample_interval - 1e-9
                        and last['pose_stage'] == 'stand'
                        and not last['faulted'] and not last['fallen'])
        first_stand = next((row['t'] - start for row in rows if row['pose_stage'] == 'stand'), None)
        results.append(dict(cycle=cycle + 1, completed=complete,
                            last_t=last['t'] if last else None,
                            last_stage=last['pose_stage'] if last else None,
                            stand_after_s=first_stand))
    return results


def override_physics(params, *, dt=None, substeps=None):
    """Override the diagnostic cadence without mutating installed defaults."""
    changes = {}
    if dt is not None:
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("dt must be finite and positive")
        changes['dt'] = dt
    if substeps is not None:
        if isinstance(substeps, bool) or not isinstance(substeps, int) or substeps < 1:
            raise ValueError("substeps must be a positive integer")
        changes['substeps'] = substeps
    return replace(params, **changes)


def advance_controlled_step(scene, command, arm_q, *, clock=time.perf_counter):
    """Time the runtime's command, kinematic arm hold and physics in order.

    A stationary arm still needs its position/velocity write on every step.
    This probe excludes DDS, telemetry and WebRTC delivery; callers separately
    include camera dispatch in their whole-step measurement.
    """
    started = clock()
    scene.go2.set_planar_velocity(*command)
    scene.go2.set_arm_q_for_metrics((arm_q.linear_m, arm_q.roll_rad,
                                     arm_q.theta1_rad, arm_q.theta2_rad))
    commanded = clock()
    scene.apply_sim_q(arm_q)
    held = clock()
    scene.step()
    stepped = clock()
    return dict(command=commanded-started, arm=held-commanded,
                physics=stepped-held, total=stepped-started)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config")
    parser.add_argument("model_bundle")
    parser.add_argument("--command", nargs=3, type=float, default=(0.15, 0.0, 0.0),
                        metavar=("VX", "VY", "WZ"))
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--stop", type=float, default=2.0)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--cycles", type=int, default=1, help="walk/stop cycles without respawn")
    parser.add_argument("--dt", type=float)
    parser.add_argument("--substeps", type=int)
    parser.add_argument("--sample-period", type=float, default=.1)
    parser.add_argument("--backend", choices=("acados", "jax_mppi"))
    parser.add_argument("--gpu-samples", type=int)
    parser.add_argument("--gpu-iterations", type=int)
    parser.add_argument("--trailing-crouch", type=float, default=0., metavar="METERS",
                        help="diagnostic additional body reference: 0..0.03 m trailing hip drop")
    parser.add_argument("--swing-reference-scale", type=float, default=1.,
                        help="diagnostic scale 0..1 of swing damping reference feedforward")
    parser.add_argument("--swing-reference-axes", choices=('all', 'vertical'), default='all')
    parser.add_argument("--aux-kv", type=float, help="diagnostic auxiliary damping override, 0..6")
    parser.add_argument("--leg-kv-damping", type=float, help="diagnostic leg damping override, 0..6")
    parser.add_argument("--stance-progress-compensation", type=float,
                        help="diagnostic 0..1 stance reference override; omitted keeps installed behavior")
    parser.add_argument("--coast-during-landing", action='store_true',
                        help="diagnostic: retain bounded current velocity during contact wait")
    parser.add_argument("--cameras", action="store_true", help="include both async render workers")
    parser.add_argument("--arm-motion", action="store_true", help="small bounded arm bend oscillations")
    parser.add_argument("--keep-target", action="store_true")
    parser.add_argument("--scene-manifest", type=Path,
                        help="import a verified scene artifact into physics and both camera replicas")
    parser.add_argument("--output", type=Path, required=True, help="JSONL sample/summary file")
    args = parser.parse_args()

    import genesis as gs
    import numpy as np
    from scipy.spatial.transform import Rotation

    from elesim_sim.runtime import AssetProcessor, GenesisApp, RuntimePrep, load_app_config
    from elesim_sim.simulation.genesis.utils import to_numpy_1d
    from elesim_sim.vision.frame_hub import FrameHub
    from elesim_protocol import default_start_sim_q

    if (not np.isfinite([*args.command, args.duration, args.settle, args.stop]).all()
            or args.duration <= 0 or min(args.settle, args.stop) < 0 or args.repeat < 1 or args.cycles < 1):
        parser.error("finite commands, positive duration/repeat and nonnegative settle/stop required")
    if args.dt is not None and (not np.isfinite(args.dt) or args.dt <= 0):
        parser.error("dt must be finite and positive")
    if not np.isfinite(args.sample_period) or args.sample_period <= 0:
        parser.error("sample period must be finite and positive")
    if not np.isfinite(args.trailing_crouch) or not 0. <= args.trailing_crouch <= .03:
        parser.error("trailing crouch must be in 0..0.03 m")
    if not np.isfinite(args.swing_reference_scale) or not 0. <= args.swing_reference_scale <= 1.:
        parser.error("swing reference scale must be in 0..1")
    for value in (args.aux_kv, args.leg_kv_damping):
        if value is not None and (not np.isfinite(value) or not 0. <= value <= 6.):
            parser.error("diagnostic damping must be finite and in 0..6")
    if args.stance_progress_compensation is not None and (
            not np.isfinite(args.stance_progress_compensation)
            or not 0. <= args.stance_progress_compensation <= 1.):
        parser.error("stance progress compensation must be in 0..1")
    bundle = load_app_config(args.config, mode="remote")
    try:
        params = override_physics(bundle.sim_param, dt=args.dt, substeps=args.substeps)
    except ValueError as exc:
        parser.error(str(exc))
    cfg = replace(bundle.sim_config, build_dir=str(Path(args.model_bundle).resolve()),
                  enable_viewer=False, sim_camera_enable=args.cameras,
                  sim_observer_camera_enable=args.cameras)
    if args.scene_manifest is not None:
        cfg = replace(cfg, scene_manifest=str(args.scene_manifest.resolve(strict=True)))
    spawn = replace(bundle.spawn_config, sim_target_enable=bool(args.keep_target))
    locomotion = bundle.go2_locomotion_config
    overrides = {key: value for key, value in (
        ("mpc_solver_backend", args.backend), ("mpc_gpu_samples", args.gpu_samples),
        ("mpc_gpu_iterations", args.gpu_iterations),
        ("mpc_aux_kv", args.aux_kv), ("mpc_leg_kv_damping", args.leg_kv_damping),
    ) if value is not None}
    locomotion = replace(locomotion, **overrides)
    hub = FrameHub(("rgbd", "hand_eye_preview", "observer")) if args.cameras else None
    app = GenesisApp(params=params, cfg=cfg, limit=bundle.joint_limit, model=spawn,
                     urdf_export_cfg=bundle.urdf_export_config,
                     go2_locomotion_config=locomotion,
                     mapping_cfg=bundle.mapping_config, frame_hub=hub)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as output:
        def emit(record):
            output.write(json.dumps(record, allow_nan=False) + "\n")
            output.flush()
            if record["kind"] != "sample":
                print(json.dumps(record), flush=True)

        failed_trials = 0
        startup_began = time.perf_counter()
        try:
            urdf = AssetProcessor(app).prepare_assets()
            runtime = RuntimePrep(app)
            from elesim_sim.simulation import scene_artifact
            import_times = []
            original_import = scene_artifact.add_scene_artifact

            def timed_scene_import(*values, **options):
                started = time.perf_counter()
                try:
                    return original_import(*values, **options)
                finally:
                    import_times.append(time.perf_counter() - started)

            # Exercise real rendering without creating a diagnostic DDS
            # participant or publishing into the installed system's topics.
            with patch.object(scene_artifact, "add_scene_artifact", timed_scene_import), patch(
                "elesim_sim.vision.sim_camera.SimCameraPublisher", return_value=None
            ):
                runtime.init_genesis(urdf, attach_scene_cameras=False)
            scene = app.sim_scene
            streams = ("hand_eye_preview", "observer")
            first_frames_wall_s = None
            if args.cameras:
                scene.configure_camera_render_worker(runtime.camera_render_spec(urdf), {
                    streams[0]: (cfg.sim_camera_width, cfg.sim_camera_height),
                    streams[1]: (cfg.sim_observer_camera_width, cfg.sim_observer_camera_height),
                }, timeout_s=cfg.camera_worker_start_timeout_s, wait=True)
                scene.configure_frame_dispatchers()
                scene.start_frame_dispatchers()
                first_frames_wall_s = wait_for_camera_frames(scene, hub, cfg)
            entity = scene.go2_entity
            controller = scene.go2._controller
            # Explicit research intervention, recorded with the trial. This
            # does not edit the installed controller or its default settings.
            from elesim_sim.robot.go2 import pympc_controller as controller_module
            original_swing_reference = controller_module.swing_damping_reference_torque
            def scaled_swing_reference(*values, **options):
                return swing_reference_for_axes(original_swing_reference, *values,
                    axes=args.swing_reference_axes, scale=args.swing_reference_scale, **options)
            controller_module.swing_damping_reference_torque = scaled_swing_reference
            latest_control = {}
            latest_actuation = {}
            original_sample = controller._sample
            original_auxiliary = controller._aux_pd_torque
            native_stance_reference = getattr(controller, '_stance_damping_reference', None)
            if native_stance_reference is not None and args.stance_progress_compensation is not None:
                def scaled_native_stance_reference(*values, **options):
                    return args.stance_progress_compensation*native_stance_reference(*values, **options)
                controller._stance_damping_reference = scaled_native_stance_reference

            def auxiliary_with_progress_reference(joint_velocity):
                torque = original_auxiliary(joint_velocity)
                if (native_stance_reference is not None
                        or not args.stance_progress_compensation or controller._settling
                        or not latest_actuation):
                    return torque
                sample = latest_actuation['sample']
                command = Rotation.from_euler('z', sample.rpy[2]).apply(
                    [controller._cmd.vx, controller._cmd.vy, 0.])
                progress = commanded_progress_velocity(latest_actuation['base_velocity'], command)
                damping = (.5+controller._config.joint_damping
                           +controller._config.leg_kv_damping+controller._config.aux_kv)
                for i in range(4):
                    if sample.contacts[i, 0] != 1.:
                        continue
                    sl = slice(3*i, 3*i+3)
                    torque[sl] += (args.stance_progress_compensation*controller._contact_gait.load[i]
                        *original_swing_reference(latest_actuation['jacobians'][i], np.zeros(3),
                            progress, np.zeros(3), damping=damping,
                            velocity_limits=controller._swing_velocity_limits[sl]))
                return torque

            controller._aux_pd_torque = auxiliary_with_progress_reference

            def observe_control_sample():
                result = original_sample()
                sample = trailing_crouch_reference(result[0],
                    command_vx=float(controller._cmd.vx), crouch_m=args.trailing_crouch)
                base_velocity = Rotation.from_quat(controller._bridge.last_q[3:7]).apply(
                    controller._bridge.last_dq[:3])
                if args.coast_during_landing:
                    command = Rotation.from_euler('z', sample.rpy[2]).apply(
                        [controller._cmd.vx, controller._cmd.vy, 0.])
                    sample = landing_velocity_reference(sample, blocked=controller._contact_gait.blocked,
                        actual_velocity=base_velocity, command_world=command,
                        horizon_s=.5*controller._solver.horizon*controller._solver.dt)
                result = (sample, *result[1:])
                latest_actuation.update(sample=sample, jacobians=result[3],
                    base_velocity=base_velocity)
                latest_control.update(
                    com=sample.com_position.tolist(),
                    desired_com=sample.desired_com_position.tolist(),
                    velocity=sample.com_velocity.tolist(),
                    desired_velocity=sample.desired_com_velocity.tolist(),
                    contacts=sample.contacts[:, 0].tolist(),
                    actuation_scales=(sample.actuation_scales.tolist()
                        if getattr(sample, 'actuation_scales', None) is not None else None),
                    feet_world=sample.feet_world.tolist(),
                    foot_velocity_world=result[2].tolist(),
                    foot_jacobians_world=result[3].tolist(),
                )
                return result

            controller._sample = observe_control_sample
            original_force_solve = controller._solver.solve
            def observe_force_solution(sample):
                forces = original_force_solve(sample)
                active = getattr(sample, 'support_torque_maps', None) is not None
                latest_control['support_motor_constraints_active'] = active
                if active:
                    loads = sample.support_loads if sample.support_loads is not None else sample.contacts[:, 0]
                    gain = sample.actuation_scales[0] if sample.actuation_scales is not None else 1.
                    applied = forces*np.asarray(loads)[:, None]*gain
                    torque = (np.einsum('ijk,ik->ij', sample.support_torque_maps[0], applied)
                              + sample.support_torque_offsets[0])
                    latest_control['planned_motor_torque_max_ratio'] = float(np.max(
                        np.abs(torque)/sample.support_torque_limits[0]))
                return forces
            controller._solver.solve = observe_force_solution
            solve_times = []
            controller._timing_sink = lambda name, seconds: (
                solve_times.append(seconds) if name == "go2_pympc_solve" else None
            )
            dt = float(params.dt)
            emit(dict(kind="configuration", dt=dt, substeps=params.substeps, command=args.command,
                      startup_wall_s=time.perf_counter()-startup_began,
                      camera_first_frames_wall_s=first_frames_wall_s,
                      scene_manifest=cfg.scene_manifest,
                      scene_revision=app.imported_scene.revision if app.imported_scene else None,
                      scene_object_count=len(app.imported_scene_entities),
                      physics_scene_import_wall_s=sum(import_times),
                      timing_clock="simulation duration; monotonic wall timing",
                      dds=False, webrtc=False,
                      solver_dt=controller._solver.dt, solver_stride=controller._solve_stride,
                      gait_hz=controller._config.gait_hz, gait_duty=controller._config.gait_duty,
                      duration=args.duration, settle=args.settle, stop=args.stop, cycles=args.cycles,
                      fixed_target=bool(args.keep_target), controller=type(controller).__name__,
                      backend=locomotion.mpc_solver_backend, gpu_samples=locomotion.mpc_gpu_samples,
                      gpu_iterations=locomotion.mpc_gpu_iterations, cameras=args.cameras,
                      diagnostic_trailing_crouch_m=args.trailing_crouch,
                      diagnostic_swing_reference_scale=args.swing_reference_scale,
                      diagnostic_swing_reference_axes=args.swing_reference_axes,
                      diagnostic_stance_progress_compensation=args.stance_progress_compensation,
                      diagnostic_coast_during_landing=args.coast_during_landing,
                      runtime_stance_reference=(native_stance_reference is not None
                          and controller._solver.backend == 'jax_mppi'),
                      runtime_motor_constraints=(hasattr(controller_module, 'support_actuator_constraints')
                          and controller._solver.backend == 'jax_mppi'),
                      runtime_trailing_crouch_m=(getattr(controller_module, '_WALK_TRAILING_CROUCH_M', 0.)
                          if controller._solver.backend == 'jax_mppi' else 0.),
                      locomotion_config=asdict(locomotion),
                      arm_motion=args.arm_motion,
                      arm_hold_every_step=True,
                      step_timing_scope="command+arm_hold+physics+camera_dispatch",
                      camera_target_hz=[cfg.sim_camera_max_hz, cfg.sim_observer_camera_max_hz] if args.cameras else None))
            for trial in range(args.repeat):
                scene.reset_environment(mapping_cfg=app._proto_cfg)
                latest_control.clear()
                began = time.perf_counter()
                first_motion_wall = None
                samples = []
                solve_times.clear()
                step_times = []
                moving_step_times = []
                section_times = {name: [] for name in ("command", "arm", "physics")}
                frame_versions = {name: hub.version(name) for name in streams} if hub else {}
                frame_counts = {name: 0 for name in streams}
                frame_ages = {name: [] for name in streams}
                command_start = None
                command_end = None
                arm_start = default_start_sim_q(app._proto_cfg)
                total_s = args.settle + args.cycles * (args.duration + args.stop)
                for step in range(int(np.ceil(total_s / dt))):
                    t = step * dt
                    phase_s = (t - args.settle) % (args.duration + args.stop)
                    moving = t >= args.settle and phase_s < args.duration
                    if moving and first_motion_wall is None:
                        first_motion_wall = time.perf_counter()
                    arm_q = arm_start
                    if args.arm_motion:
                        arm_q = replace(arm_start,
                            theta1_rad=arm_start.theta1_rad + .12*np.sin(2*np.pi*t/8.),
                            theta2_rad=arm_start.theta2_rad + .08*np.sin(2*np.pi*t/8.))
                    step_started = time.perf_counter()
                    section_cost = advance_controlled_step(
                        scene, args.command if moving else (0., 0., 0.), arm_q)
                    for name in section_times:
                        section_times[name].append(section_cost[name])
                    if hub is not None:
                        scene.maybe_publish_camera(arm_q=None, max_hz=cfg.sim_camera_max_hz,
                                                   sim_time_s=t, rgb_enabled=cfg.sim_camera_rgb,
                                                   depth_enabled=cfg.sim_camera_depth)
                        scene.maybe_publish_observer_camera(max_hz=cfg.sim_observer_camera_max_hz,
                                                            sim_time_s=t)
                        for name in streams:
                            version = hub.version(name)
                            if version != frame_versions[name]:
                                frame_counts[name] += version - frame_versions[name]
                                frame_versions[name] = version
                                frame = hub.latest(name)
                                frame_ages[name].append(max(0., time.time()-frame.ts))
                    elapsed_step = time.perf_counter() - step_started
                    step_times.append(elapsed_step)
                    if moving:
                        moving_step_times.append(elapsed_step)
                    if step % max(1, round(args.sample_period / dt)):
                        continue
                    pos = to_numpy_1d(entity.get_pos())
                    quat = to_numpy_1d(entity.get_quat())
                    if not np.isfinite(np.r_[pos, quat]).all():
                        emit(dict(kind="nonfinite", trial=trial, t=t))
                        break
                    rpy = Rotation.from_quat(quat[[1, 2, 3, 0]]).as_euler("xyz")
                    fallen = bool(pos[2] < 0.16 or np.max(np.abs(rpy[:2])) > 0.85)
                    row = dict(kind="sample", trial=trial, t=t, position=pos.tolist(),
                               rpy=rpy.tolist(), fallen=fallen,
                               pose_stage=getattr(controller, "_pose_stage", None),
                               base_twist=(controller._bridge.last_dq[:6].tolist()
                                           if controller._bridge.last_dq is not None else None),
                               faulted=bool(getattr(controller, "_faulted", False)),
                               fault_reason=str(getattr(controller, "fault_reason", "")),
                               model_mass_kg=float(controller._data.Ig.mass),
                               model_inertia_trace=float(np.trace(controller._data.Ig.inertia)),
                               feet=[to_numpy_1d(entity.get_link(leg + "_foot").get_pos()).tolist()
                                     for leg in ("FL", "FR", "RL", "RR")])
                    if controller._bridge.last_q is not None:
                        row["leg_control"] = dict(
                            q=controller._bridge.last_q[7:19].tolist(),
                            dq=controller._bridge.last_dq[6:18].tolist(),
                            tau_raw=controller._tau_raw.tolist(),
                            tau_limited=controller._tau_limited.tolist(),
                            swing_starts=controller._swing_starts.tolist(),
                            touchdown_depths=controller._touchdown_depths.tolist(),
                            touchdowns=controller._touchdowns.tolist(),
                            search_targets=controller._search_targets.tolist(),
                        )
                    if latest_control:
                        row["centroidal_control"] = dict(
                            **latest_control,
                            requested_forces=controller._force_requested.tolist(),
                            applied_forces=(controller._forces * controller._contact_gait.load[:, None]
                                * (latest_control.get('actuation_scales') or [1.])[0]).tolist(),
                        )
                    gate = getattr(controller, "_contact_gait", None)
                    if gate is not None:
                        row["contact_gate"] = dict(
                            time=gate.time, blocked=gate.blocked,
                            confirmed=gate.confirmed.tolist(), search=gate.search.tolist(),
                            load=gate.load.tolist(), missing_age=gate.missing_age.tolist(),
                            normal_forces=controller._normal_contact_forces().tolist(),
                            observed_normal=gate.observed_normal.tolist(),
                            observed_heights=gate.observed_heights.tolist(),
                            contact_age=gate.contact_age.tolist(),
                            swing_progress=gate.swing_progress.tolist(),
                        )
                    emit(row)
                    samples.append(row)
                    if moving:
                        if command_start is None:
                            command_start = pos.copy()
                        command_end = pos.copy()
                    if fallen or row["faulted"]:
                        break
                displacement = (command_end - command_start).tolist() if command_start is not None else None
                completed = bool(samples and samples[-1]["t"] >= total_s - 0.11
                                 and not samples[-1]["fallen"] and not samples[-1]["faulted"])
                progress_ok = has_commanded_progress(args.command, displacement,
                                                     duration=args.duration * args.cycles)
                completed = completed and progress_ok
                cycle_stops = stop_cycle_results(
                    samples, settle=args.settle, duration=args.duration, stop=args.stop,
                    cycles=args.cycles, sample_interval=dt * max(1, round(args.sample_period / dt)),
                )
                completed = completed and all(row['completed'] for row in cycle_stops)
                wall_s = time.perf_counter() - began
                sim_elapsed_s = len(step_times) * dt
                if hub is not None:
                    completed = (completed and all(frame_counts[name] > 0 for name in streams)
                                 and not scene.camera_render_worker.failure)
                failed_trials += int(not completed)
                emit(dict(kind="summary", trial=trial, samples=len(samples),
                          completed=completed,
                          sim_elapsed_s=sim_elapsed_s, real_time_factor=sim_elapsed_s/wall_s,
                          whole_loop_step_ms=1000*wall_s/len(step_times) if step_times else None,
                          warmup_wall_s=(first_motion_wall-began) if first_motion_wall else wall_s,
                          steady_wall_s=(began+wall_s-first_motion_wall) if first_motion_wall else None,
                          warmup_sim_s=min(args.settle, sim_elapsed_s),
                          steady_sim_s=max(0., sim_elapsed_s-args.settle),
                          whole_loop_scope="includes diagnostic state reads and JSON writes; excludes scene startup/reset",
                          commanded_progress_ok=progress_ok,
                          cycle_stops=cycle_stops,
                          wall_s=wall_s, displacement=displacement,
                          rendered_fps={name: frame_counts[name]/wall_s for name in streams} if hub else None,
                          capture_to_observation_ms_p95={name: float(np.percentile(frame_ages[name], 95)*1000)
                              if frame_ages[name] else None for name in streams} if hub else None,
                          solve_ms_p50_p95=(np.percentile(solve_times, [50, 95])*1000).tolist() if solve_times else None,
                          step_ms_p50_p95=(np.percentile(step_times, [50, 95])*1000).tolist() if step_times else None,
                          moving_step_ms_p50_p95=(np.percentile(moving_step_times, [50, 95])*1000).tolist() if moving_step_times else None,
                          section_ms_mean_p50_p95={name: [float(np.mean(values))*1000,
                              *(np.percentile(values, [50, 95])*1000).tolist()]
                              for name, values in section_times.items() if values},
                          max_abs_roll_pitch=np.max(np.abs([r["rpy"][:2] for r in samples]), axis=0).tolist() if samples else None,
                          last=samples[-1] if samples else None))
        finally:
            if 'original_swing_reference' in locals():
                controller_module.swing_damping_reference_torque = original_swing_reference
            app.sim_scene.close_frame_dispatchers()
            app.sim_scene.close_camera_publishers()
            gs.destroy()
        if failed_trials:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
