"""Reproduce GO2 walking in an isolated production Genesis scene, without DDS.

Run inside the installed Sim/development environment. The physical GO2+arm,
MPC and reset paths are the application's own implementations. Cameras are
disabled by default; --cameras includes the production render/dispatch workers
(not WebRTC encoding or network delivery). The fixed perception target is omitted so it cannot
obstruct the walking path. This never commands a connected Robot or live Sim.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import time
from unittest.mock import patch


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
    parser.add_argument("--cameras", action="store_true", help="include both async render workers")
    parser.add_argument("--arm-motion", action="store_true", help="small bounded arm bend oscillations")
    parser.add_argument("--keep-target", action="store_true")
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
    bundle = load_app_config(args.config, mode="remote")
    try:
        params = override_physics(bundle.sim_param, dt=args.dt, substeps=args.substeps)
    except ValueError as exc:
        parser.error(str(exc))
    cfg = replace(bundle.sim_config, build_dir=str(Path(args.model_bundle).resolve()),
                  enable_viewer=False, sim_camera_enable=args.cameras,
                  sim_observer_camera_enable=args.cameras)
    spawn = replace(bundle.spawn_config, sim_target_enable=bool(args.keep_target))
    locomotion = bundle.go2_locomotion_config
    overrides = {key: value for key, value in (
        ("mpc_solver_backend", args.backend), ("mpc_gpu_samples", args.gpu_samples),
        ("mpc_gpu_iterations", args.gpu_iterations),
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
        try:
            urdf = AssetProcessor(app).prepare_assets()
            runtime = RuntimePrep(app)
            # Exercise real rendering without creating a diagnostic DDS
            # participant or publishing into the installed system's topics.
            with patch("elesim_sim.vision.sim_camera.SimCameraPublisher", return_value=None):
                runtime.init_genesis(urdf, attach_scene_cameras=False)
            scene = app.sim_scene
            streams = ("hand_eye_preview", "observer")
            if args.cameras:
                scene.configure_camera_render_worker(runtime.camera_render_spec(urdf), {
                    streams[0]: (cfg.sim_camera_width, cfg.sim_camera_height),
                    streams[1]: (cfg.sim_observer_camera_width, cfg.sim_observer_camera_height),
                }, timeout_s=cfg.camera_worker_start_timeout_s, wait=True)
                scene.configure_frame_dispatchers()
                scene.start_frame_dispatchers()
            entity = scene.go2_entity
            controller = scene.go2._controller
            latest_control = {}
            original_sample = controller._sample

            def observe_control_sample():
                result = original_sample()
                sample = result[0]
                latest_control.update(
                    com=sample.com_position.tolist(),
                    desired_com=sample.desired_com_position.tolist(),
                    velocity=sample.com_velocity.tolist(),
                    desired_velocity=sample.desired_com_velocity.tolist(),
                )
                return result

            controller._sample = observe_control_sample
            solve_times = []
            controller._timing_sink = lambda name, seconds: (
                solve_times.append(seconds) if name == "go2_pympc_solve" else None
            )
            dt = float(params.dt)
            emit(dict(kind="configuration", dt=dt, substeps=params.substeps, command=args.command,
                      solver_dt=controller._solver.dt, solver_stride=controller._solve_stride,
                      duration=args.duration, settle=args.settle, stop=args.stop, cycles=args.cycles,
                      fixed_target=bool(args.keep_target), controller=type(controller).__name__,
                      backend=locomotion.mpc_solver_backend, gpu_samples=locomotion.mpc_gpu_samples,
                      gpu_iterations=locomotion.mpc_gpu_iterations, cameras=args.cameras,
                      arm_motion=args.arm_motion,
                      camera_target_hz=[cfg.sim_camera_max_hz, cfg.sim_observer_camera_max_hz] if args.cameras else None))
            for trial in range(args.repeat):
                scene.reset_environment(mapping_cfg=app._proto_cfg)
                latest_control.clear()
                began = time.perf_counter()
                samples = []
                solve_times.clear()
                step_times = []
                moving_step_times = []
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
                    scene.go2.set_planar_velocity(*(args.command if moving else (0.0, 0.0, 0.0)))
                    if args.arm_motion:
                        scene.apply_sim_q(replace(arm_start,
                            theta1_rad=arm_start.theta1_rad + .12*np.sin(2*np.pi*t/8.),
                            theta2_rad=arm_start.theta2_rad + .08*np.sin(2*np.pi*t/8.)))
                    step_started = time.perf_counter()
                    scene.step()
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
                            touchdowns=controller._touchdowns.tolist(),
                            search_targets=controller._search_targets.tolist(),
                        )
                    if latest_control:
                        row["centroidal_control"] = dict(
                            **latest_control,
                            requested_forces=controller._force_requested.tolist(),
                            applied_forces=(controller._forces * controller._contact_gait.load[:, None]).tolist(),
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
                if hub is not None:
                    completed = (completed and all(frame_counts[name] > 0 for name in streams)
                                 and not scene.camera_render_worker.failure)
                failed_trials += int(not completed)
                emit(dict(kind="summary", trial=trial, samples=len(samples),
                          completed=completed,
                          commanded_progress_ok=progress_ok,
                          cycle_stops=cycle_stops,
                          wall_s=wall_s, displacement=displacement,
                          rendered_fps={name: frame_counts[name]/wall_s for name in streams} if hub else None,
                          capture_to_observation_ms_p95={name: float(np.percentile(frame_ages[name], 95)*1000)
                              if frame_ages[name] else None for name in streams} if hub else None,
                          solve_ms_p50_p95=(np.percentile(solve_times, [50, 95])*1000).tolist() if solve_times else None,
                          step_ms_p50_p95=(np.percentile(step_times, [50, 95])*1000).tolist() if step_times else None,
                          moving_step_ms_p50_p95=(np.percentile(moving_step_times, [50, 95])*1000).tolist() if moving_step_times else None,
                          max_abs_roll_pitch=np.max(np.abs([r["rpy"][:2] for r in samples]), axis=0).tolist() if samples else None,
                          last=samples[-1] if samples else None))
        finally:
            app.sim_scene.close_frame_dispatchers()
            app.sim_scene.close_camera_publishers()
            gs.destroy()
        if failed_trials:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
