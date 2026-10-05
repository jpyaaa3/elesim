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
from pathlib import Path
import time
from unittest.mock import patch


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
    bundle = load_app_config(args.config, mode="remote")
    params = bundle.sim_param if args.dt is None else replace(bundle.sim_param, dt=args.dt)
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
            solve_times = []
            controller._timing_sink = lambda name, seconds: (
                solve_times.append(seconds) if name == "go2_pympc_solve" else None
            )
            dt = float(params.dt)
            emit(dict(kind="configuration", dt=dt, command=args.command,
                      duration=args.duration, settle=args.settle, stop=args.stop, cycles=args.cycles,
                      fixed_target=bool(args.keep_target), controller=type(controller).__name__,
                      backend=locomotion.mpc_solver_backend, gpu_samples=locomotion.mpc_gpu_samples,
                      gpu_iterations=locomotion.mpc_gpu_iterations, cameras=args.cameras,
                      arm_motion=args.arm_motion,
                      camera_target_hz=[cfg.sim_camera_max_hz, cfg.sim_observer_camera_max_hz] if args.cameras else None))
            for trial in range(args.repeat):
                scene.reset_environment(mapping_cfg=app._proto_cfg)
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
                    if step % max(1, round(0.1 / dt)):
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
                if args.stop > 0:
                    completed = completed and samples[-1]["pose_stage"] == "stand"
                wall_s = time.perf_counter() - began
                if hub is not None:
                    completed = (completed and all(frame_counts[name] > 0 for name in streams)
                                 and not scene.camera_render_worker.failure)
                failed_trials += int(not completed)
                emit(dict(kind="summary", trial=trial, samples=len(samples),
                          completed=completed,
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
