from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .iklib import aligner as ik_aligner
from .iklib import kinematics as ik_kin
from .iklib import solver as ik_solver
from .iklib import tweaker as ik_tweaker


@dataclass(frozen=True)
class SolveAndTweakResult:
    success: bool
    q: Optional[np.ndarray]
    position_error_m: float
    seed_name: str
    iterations: int
    reason: str = ""


def load_solver_context(config_path: str):
    return ik_solver.load_solver_context(config_path)


def solve_then_tweak(
    *,
    target_world: Sequence[float],
    target_dir_world: Optional[Sequence[float]],
    context: dict,
    position_tol_m: float,
    max_iters: int,
    current_seed: Sequence[float],
    tweak_position_hold_tol_m: float = 1.5e-2,
    tweak_rounds: int = 10,
) -> SolveAndTweakResult:
    result = ik_solver.solve_ik(
        target_world=target_world,
        context=context,
        position_tol_m=position_tol_m,
        max_iters=max_iters,
        current_seed=current_seed,
    )
    if (not result.success) or result.q is None:
        return SolveAndTweakResult(
            success=False,
            q=None if result.q is None else np.asarray(result.q, dtype=float).reshape(4).copy(),
            position_error_m=float(result.position_error_m),
            seed_name=str(result.seed_name),
            iterations=int(result.iterations),
            reason=str(result.reason),
        )

    q = np.asarray(result.q, dtype=float).reshape(4).copy()
    err_m = float(result.position_error_m)
    if target_dir_world is not None:
        direction = np.asarray(target_dir_world, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(direction))
        if dnorm > 1e-9:
            hold_target = ik_kin._forward_grasp_world(context, q)
            refine = ik_aligner.refine_direction_with_position_hold(
                current_q=q,
                target_world=hold_target,
                target_dir_world=(direction / dnorm),
                context=context,
                position_hold_tol_m=tweak_position_hold_tol_m,
                rounds=tweak_rounds,
            )
            q = np.asarray(refine.q, dtype=float).reshape(4).copy()
            err_m = float(refine.position_error_m)

    return SolveAndTweakResult(
        success=True,
        q=q,
        position_error_m=err_m,
        seed_name=str(result.seed_name),
        iterations=int(result.iterations),
        reason="converged",
    )


def tweak_only(
    *,
    current_q: Sequence[float],
    hold_target_world: Optional[Sequence[float]],
    target_dir_world: Sequence[float],
    context: dict,
    position_hold_tol_m: float = 5e-3,
    rounds: int = 10,
) -> ik_tweaker.TweakResult:
    q = np.asarray(current_q, dtype=float).reshape(4)
    hold_target = None if hold_target_world is None else np.asarray(hold_target_world, dtype=float).reshape(3)
    if hold_target is None:
        hold_target = ik_kin._forward_grasp_world(context, q)
    return ik_tweaker.tweak_pose(
        current_q=q,
        target_world=hold_target,
        target_dir_world=target_dir_world,
        context=context,
        position_tol_m=position_hold_tol_m,
        max_iters=rounds,
    )


__all__ = [
    "SolveAndTweakResult",
    "load_solver_context",
    "solve_then_tweak",
    "tweak_only",
]
