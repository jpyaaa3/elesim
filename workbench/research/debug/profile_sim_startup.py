"""Compare Genesis array modes with the same robot and physical solver.

Run in an installed Sim/development environment. This starts no DDS peer and
does not command the running simulation. The scene omits cameras, target and
MPC: its times describe physics startup, not full application readiness.
Use separate processes for each run, with the same persistent cache directory.
"""

from __future__ import annotations

import argparse
import cProfile
import pstats
import time


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", help="GO2+arm robot.urdf in the installed model bundle")
    parser.add_argument("--performance-mode", action="store_true")
    parser.add_argument("--profile", action="store_true", help="adds profiler overhead")
    args = parser.parse_args()

    import genesis as gs
    import numpy as np
    import torch

    from elesim_sim.robot.go2.initial_pose import prepare_neutral_stand_pose

    started = time.perf_counter()
    gs.init(backend=gs.gpu, logging_level="warning", performance_mode=args.performance_mode)
    print("INIT", time.perf_counter() - started, "NDARRAY", gs.use_ndarray, flush=True)
    try:
        scene = gs.Scene(
            show_viewer=False,
            sim_options=gs.options.SimOptions(dt=0.02, substeps=1, gravity=(0, 0, -9.81)),
            rigid_options=gs.options.RigidOptions(
                constraint_solver=gs.constraint_solver.Newton, iterations=50, noslip_iterations=0,
            ),
        )
        scene.add_entity(gs.morphs.Plane())
        started = time.perf_counter()
        entity = scene.add_entity(gs.morphs.URDF(
            file=args.urdf, pos=(0, 0, 0.4), fixed=False, merge_fixed_links=False,
            prioritize_urdf_material=True, default_armature=0.0,
        ))
        prepare_neutral_stand_pose(entity)
        print("MODEL", time.perf_counter() - started, flush=True)
        profile = cProfile.Profile()
        started = time.perf_counter()
        if args.profile:
            profile.enable()
        try:
            scene.build()
        finally:
            profile.disable()
        print("BUILD", time.perf_counter() - started, flush=True)
        if args.profile:
            pstats.Stats(profile).strip_dirs().sort_stats("cumulative").print_stats(22)

        # Uncontrolled physics is only a throughput/finite-state probe. This
        # does not establish standing stability, MPC or walking correctness.
        durations = []
        for _ in range(110):
            started = time.perf_counter()
            scene.step()
            torch.cuda.synchronize()
            durations.append(time.perf_counter() - started)
        print("STEP_MS_P50_P95", np.percentile(durations[10:], [50, 95]) * 1000, flush=True)
        print("FINITE_QPOS", bool(np.isfinite(entity.get_qpos().cpu().numpy()).all()), flush=True)
    finally:
        gs.destroy()


if __name__ == "__main__":
    main()
