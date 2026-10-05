"""Reproduce GO2 walking in an isolated production Genesis scene, without DDS.

Run inside the installed Sim/development environment. The physical GO2+arm,
MPC and reset paths are the application's own implementations. Cameras are
disabled and the fixed perception target is omitted by default so it cannot
obstruct the walking path. This never commands a connected Robot or live Sim.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import time


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
    parser.add_argument("--keep-target", action="store_true")
    parser.add_argument("--output", type=Path, required=True, help="JSONL sample/summary file")
    args = parser.parse_args()

    import genesis as gs
    import numpy as np
    from scipy.spatial.transform import Rotation

    from elesim_sim.runtime import AssetProcessor, GenesisApp, RuntimePrep, load_app_config
    from elesim_sim.simulation.genesis.utils import to_numpy_1d

    if (not np.isfinite([*args.command, args.duration, args.settle, args.stop]).all()
            or args.duration <= 0 or min(args.settle, args.stop) < 0 or args.repeat < 1 or args.cycles < 1):
        parser.error("finite commands, positive duration/repeat and nonnegative settle/stop required")
    if args.dt is not None and (not np.isfinite(args.dt) or args.dt <= 0):
        parser.error("dt must be finite and positive")
    bundle = load_app_config(args.config, mode="remote")
    params = bundle.sim_param if args.dt is None else replace(bundle.sim_param, dt=args.dt)
    cfg = replace(bundle.sim_config, build_dir=str(Path(args.model_bundle).resolve()),
                  enable_viewer=False, sim_camera_enable=False, sim_observer_camera_enable=False)
    spawn = replace(bundle.spawn_config, sim_target_enable=bool(args.keep_target))
    app = GenesisApp(params=params, cfg=cfg, limit=bundle.joint_limit, model=spawn,
                     urdf_export_cfg=bundle.urdf_export_config,
                     go2_locomotion_config=bundle.go2_locomotion_config,
                     mapping_cfg=bundle.mapping_config)
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
            RuntimePrep(app).init_genesis(urdf, attach_scene_cameras=False)
            scene = app.sim_scene
            entity = scene.go2_entity
            controller = scene.go2._controller
            dt = float(params.dt)
            emit(dict(kind="configuration", dt=dt, command=args.command,
                      duration=args.duration, settle=args.settle, stop=args.stop, cycles=args.cycles,
                      fixed_target=bool(args.keep_target), controller=type(controller).__name__))
            for trial in range(args.repeat):
                scene.reset_environment(mapping_cfg=app._proto_cfg)
                began = time.perf_counter()
                samples = []
                command_start = None
                command_end = None
                total_s = args.settle + args.cycles * (args.duration + args.stop)
                for step in range(int(np.ceil(total_s / dt))):
                    t = step * dt
                    phase_s = (t - args.settle) % (args.duration + args.stop)
                    moving = t >= args.settle and phase_s < args.duration
                    scene.go2.set_planar_velocity(*(args.command if moving else (0.0, 0.0, 0.0)))
                    scene.step()
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
                failed_trials += int(not completed)
                emit(dict(kind="summary", trial=trial, samples=len(samples),
                          completed=completed,
                          wall_s=time.perf_counter() - began, displacement=displacement,
                          max_abs_roll_pitch=np.max(np.abs([r["rpy"][:2] for r in samples]), axis=0).tolist() if samples else None,
                          last=samples[-1] if samples else None))
        finally:
            gs.destroy()
        if failed_trials:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
