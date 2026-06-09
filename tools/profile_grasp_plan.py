#!/usr/bin/env python3
"""Profile Grasp Plan (kinematic IK chain) compute time.

Compute-only (no host/sim):
  ELESIM_PROFILE_PICK=1 python tools/profile_grasp_plan.py

Compare full vs lite align for planning:
  ELESIM_PROFILE_PICK=1 python tools/profile_grasp_plan.py --compare-align

With cProfile top functions:
  ELESIM_PROFILE_PICK=1 python tools/profile_grasp_plan.py --cprofile
"""

from __future__ import annotations

import argparse
import cProfile
import os
import pstats
import sys
import time
from io import StringIO
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from engine import ik as ik_pipeline
from engine.iklib.kinematics import _ReachModel
from engine.profile.pick_timing import (
    GraspPlanStats,
    PickTimingCollector,
    enabled as pick_profile_enabled,
    format_grasp_plan_report,
    GraspPlanProfile,
    fk_call_count,
    install_fk_counter,
    reset_fk_count,
    uninstall_fk_counter,
)
from engine.visual_servoing.grasp_trajectory import (
    plan_grasp_approach_trajectory,
    plan_grasp_feasible_trajectory,
)


def _build_ik_fk(
    *,
    ik_context: dict,
    sag_model: dict,
    ik_cfg,
    pick_cfg,
    align_mode: str,
    timing: PickTimingCollector | None,
) -> tuple:
    ctx = dict(ik_context)
    ctx["sag_model"] = dict(sag_model)
    ik_kwargs = {
        "context": ctx,
        "position_tol_m": max(float(ik_cfg.tol), 1e-4),
        "max_iters": max(int(ik_cfg.max_iters), 1),
        "align_mode": str(align_mode),
        "align_skip_under_deg": float(pick_cfg.ik_align_skip_under_deg),
        "tweak_rounds": max(int(pick_cfg.ik_align_rounds), 1),
    }
    model = _ReachModel(context=ctx, limit=ctx["limit"])
    ik_success = {"n": 0}

    def ik_fn(**kwargs):
        merged = dict(ik_kwargs)
        merged.update(kwargs)
        if timing is not None:
            timing.ik_calls += 1
            merged["timing"] = timing
        result = ik_pipeline.solve_then_align(**merged)
        if timing is not None and bool(getattr(result, "success", False)):
            ik_success["n"] += 1
        return result

    def fk_fn(q):
        q4 = np.asarray(q, dtype=float).reshape(4)
        pos = np.asarray(model.grasp_position(q4), dtype=float).reshape(3)
        direc = np.asarray(model.grasp_direction(q4), dtype=float).reshape(3)
        return type(
            "FkTip",
            (),
            {
                "position_world": (float(pos[0]), float(pos[1]), float(pos[2])),
                "direction_world": (float(direc[0]), float(direc[1]), float(direc[2])),
            },
        )()

    ik_fn.ik_success_counter = ik_success  # type: ignore[attr-defined]
    return ik_fn, fk_fn


def _run_plan_once(
    *,
    label: str,
    config_path: str,
    object_xyz: tuple[float, float, float],
    q_seed: tuple[float, float, float, float],
    approach_dir: tuple[float, float, float],
    align_mode: str,
) -> GraspPlanProfile:
    os.environ.setdefault("ELESIM_PROFILE_PICK", "1")
    bundle, ik_context = ik_pipeline.load_solver_context(config_path)
    pk = bundle.pick_config
    ik_cfg = bundle.ik_config
    sag_model: dict = {}

    step_m = float(max(pk.grasp_waypoint_step_m, 0.005))
    blind_start_m = float(max(pk.grasp_blind_start_m, 0.0))
    max_waypoints = max(1, int(pk.grasp_max_waypoints))
    standoff_m = float(max(pk.grasp_standoff_m, 0.0))
    dir_u = np.asarray(approach_dir, dtype=float).reshape(3)
    dir_u = dir_u / max(float(np.linalg.norm(dir_u)), 1e-9)
    dir3 = (float(dir_u[0]), float(dir_u[1]), float(dir_u[2]))
    obj = tuple(float(v) for v in object_xyz)
    nominal = tuple(
        float(v)
        for v in (
            np.asarray(obj, dtype=float).reshape(3)
            - dir_u * standoff_m
        )
    )

    timing = PickTimingCollector()
    stats = GraspPlanStats()
    install_fk_counter()
    reset_fk_count()
    t0 = time.perf_counter()

    ik_fn, fk_fn = _build_ik_fk(
        ik_context=ik_context,
        sag_model=sag_model,
        ik_cfg=ik_cfg,
        pick_cfg=pk,
        align_mode=align_mode,
        timing=timing,
    )
    q_arr = np.asarray(q_seed, dtype=float).reshape(4)
    traj_start = tuple(float(v) for v in fk_fn(q_arr).position_world)

    with timing.span("geom_plan"):
        geom_plan = plan_grasp_approach_trajectory(
            start_position=traj_start,
            end_position=nominal,
            start_direction=dir3,
            end_direction=dir3,
            object_world=obj,
            step_m=step_m,
            blind_start_m=blind_start_m,
            grasp_standoff_m=standoff_m,
            max_waypoints=max_waypoints,
        )
    t_geom_s = timing.get("geom_plan")

    with timing.span("kinematic_plan"):
        waypoints = plan_grasp_feasible_trajectory(
            start_position=traj_start,
            end_position=nominal,
            start_direction=dir3,
            end_direction=dir3,
            object_world=obj,
            q_seed=q_arr,
            step_m=step_m,
            blind_start_m=blind_start_m,
            ik_fn=ik_fn,
            fk_fn=fk_fn,
            grasp_standoff_m=standoff_m,
            max_waypoints=max_waypoints,
            max_dir_error_deg=float(pk.grasp_waypoint_max_dir_error_deg),
            max_approach_drift_deg=float(pk.grasp_waypoint_max_approach_drift_deg),
            stats=stats,
        )
    t_kinematic_s = timing.get("kinematic_plan")
    t_total = time.perf_counter() - t0

    profile = GraspPlanProfile(
        phase=f"grasp_plan:{label}",
        t_total_s=float(t_total),
        t_geom_s=float(t_geom_s),
        t_kinematic_s=float(t_kinematic_s),
        t_solve_position_s=timing.get("solve_position"),
        t_align_s=timing.get("align_direction"),
        ik_calls=int(timing.ik_calls),
        ik_success=int(ik_fn.ik_success_counter["n"]),  # type: ignore[attr-defined]
        fk_calls=fk_call_count(),
        waypoints=len(waypoints),
        geom_waypoints=len(geom_plan),
        stats=stats,
        success=bool(waypoints),
    )
    uninstall_fk_counter()
    return profile


def main() -> int:
    ap = argparse.ArgumentParser(description="Profile grasp kinematic plan (compute-only).")
    ap.add_argument("--config", default=str(ROOT / "config.ini"))
    ap.add_argument(
        "--object",
        nargs=3,
        type=float,
        default=(0.33, 0.01, 0.92),
        metavar=("X", "Y", "Z"),
    )
    ap.add_argument(
        "--q-seed",
        nargs=4,
        type=float,
        default=(0.0, 0.0, 0.0, 0.0),
        metavar=("LIN", "ROLL", "S1", "S2"),
    )
    ap.add_argument(
        "--approach-dir",
        nargs=3,
        type=float,
        default=(1.0, 0.0, 0.0),
        metavar=("DX", "DY", "DZ"),
    )
    ap.add_argument(
        "--align",
        choices=("full", "lite"),
        default="full",
        help="align mode (production plan uses force_full=True → full)",
    )
    ap.add_argument(
        "--compare-align",
        action="store_true",
        help="run full and lite align back-to-back",
    )
    ap.add_argument("--cprofile", action="store_true")
    args = ap.parse_args()

    if not pick_profile_enabled():
        print("[profile-grasp] hint: ELESIM_PROFILE_PICK=1 enables span report in UI too")

    object_xyz = tuple(float(v) for v in args.object)
    q_seed = tuple(float(v) for v in args.q_seed)
    approach_dir = tuple(float(v) for v in args.approach_dir)

    def _run(label: str, align: str) -> GraspPlanProfile:
        if args.cprofile:
            pr = cProfile.Profile()
            pr.enable()
        profile = _run_plan_once(
            label=label,
            config_path=str(args.config),
            object_xyz=object_xyz,
            q_seed=q_seed,
            approach_dir=approach_dir,
            align_mode=align,
        )
        if args.cprofile:
            pr.disable()
            stream = StringIO()
            pstats.Stats(pr, stream=stream).sort_stats("cumulative").print_stats(25)
            print(stream.getvalue())
        print(format_grasp_plan_report(profile))
        return profile

    if args.compare_align:
        _run("full", "full")
        print("")
        _run("lite", "lite")
        return 0

    _run(str(args.align), str(args.align))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
