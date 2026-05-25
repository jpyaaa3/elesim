from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .iklib import aligner as ik_aligner
from .iklib import kinematics as ik_kin
from .iklib import solver as ik_solver
from .iklib import tweaker as ik_tweaker


@dataclass(frozen=True)
class SolveAndAlignResult:
    success: bool
    q: Optional[np.ndarray]
    position_error_m: float
    seed_name: str
    iterations: int
    align_attempted: bool = False
    align_position_kept: bool = False
    align_direction_improved: bool = False
    direction_angle_rad: float = 0.0
    initial_direction_angle_rad: float = 0.0
    reason: str = ""


def load_solver_context(config_path: str):
    return ik_solver.load_solver_context(config_path)


def solve_then_align(
    *,
    target_world: Sequence[float],
    target_dir_world: Optional[Sequence[float]],
    context: dict,
    position_tol_m: float,
    max_iters: int,
    current_seed: Sequence[float],
    tweak_position_hold_tol_m: float = 1.5e-2,
    tweak_rounds: int = 10,
) -> SolveAndAlignResult:
    result = ik_solver.solve_ik(
        target_world=target_world,
        context=context,
        position_tol_m=position_tol_m,
        max_iters=max_iters,
        current_seed=current_seed,
    )
    if (not result.success) or result.q is None:
        return SolveAndAlignResult(
            success=False,
            q=None if result.q is None else np.asarray(result.q, dtype=float).reshape(4).copy(),
            position_error_m=float(result.position_error_m),
            seed_name=str(result.seed_name),
            iterations=int(result.iterations),
            reason=str(result.reason),
        )

    q = np.asarray(result.q, dtype=float).reshape(4).copy()
    err_m = float(result.position_error_m)
    align_attempted = False
    align_position_kept = False
    align_direction_improved = False
    direction_angle_rad = 0.0
    initial_direction_angle_rad = 0.0
    if target_dir_world is not None:
        direction = np.asarray(target_dir_world, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(direction))
        if dnorm > 1e-9:
            align_attempted = True
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
            align_position_kept = bool(refine.position_kept)
            align_direction_improved = bool(refine.direction_improved)
            direction_angle_rad = float(refine.direction_angle_rad)
            initial_direction_angle_rad = float(refine.initial_direction_angle_rad)

    reason = "position_converged"
    if align_attempted:
        if align_position_kept and align_direction_improved:
            reason = "position_converged_align_improved"
        elif align_position_kept:
            reason = "position_converged_align_no_improvement"
        else:
            reason = "position_converged_align_rejected"

    return SolveAndAlignResult(
        success=True,
        q=q,
        position_error_m=err_m,
        seed_name=str(result.seed_name),
        iterations=int(result.iterations),
        align_attempted=align_attempted,
        align_position_kept=align_position_kept,
        align_direction_improved=align_direction_improved,
        direction_angle_rad=direction_angle_rad,
        initial_direction_angle_rad=initial_direction_angle_rad,
        reason=reason,
    )


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
) -> SolveAndAlignResult:
    return solve_then_align(
        target_world=target_world,
        target_dir_world=target_dir_world,
        context=context,
        position_tol_m=position_tol_m,
        max_iters=max_iters,
        current_seed=current_seed,
        tweak_position_hold_tol_m=tweak_position_hold_tol_m,
        tweak_rounds=tweak_rounds,
    )


def tweak_only(
    *,
    current_q: Sequence[float],
    hold_target_world: Optional[Sequence[float]],
    target_dir_world: Sequence[float],
    context: dict,
    actual_tip_world: Optional[Sequence[float]] = None,
    actual_dir_world: Optional[Sequence[float]] = None,
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
        actual_tip_world=actual_tip_world,
        actual_dir_world=actual_dir_world,
        position_tol_m=position_hold_tol_m,
        max_iters=rounds,
    )


def compute_tweak_step(
    *,
    current_q: Sequence[float],
    target_world: Sequence[float],
    target_dir_world: Sequence[float],
    context: dict,
    actual_tip_world: Optional[Sequence[float]] = None,
    actual_dir_world: Optional[Sequence[float]] = None,
    step_scale: float = 1.0,
):
    return ik_tweaker.compute_tweak_step(
        current_q=current_q,
        target_world=target_world,
        target_dir_world=target_dir_world,
        context=context,
        actual_tip_world=actual_tip_world,
        actual_dir_world=actual_dir_world,
        step_scale=step_scale,
    )


__all__ = [
    "SolveAndAlignResult",
    "load_solver_context",
    "solve_then_align",
    "solve_then_tweak",
    "compute_tweak_step",
    "tweak_only",
]


# Backward-compatibility alias for older callers.
SolveAndTweakResult = SolveAndAlignResult
