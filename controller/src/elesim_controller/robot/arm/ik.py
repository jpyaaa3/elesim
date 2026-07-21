from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, Optional, Sequence

import numpy as np

if TYPE_CHECKING:
    from elesim_controller.observability.pick_timing import PickTimingCollector

from .iklib import aligner as ik_aligner
from .iklib import kinematics as ik_kin
from .iklib import solver as ik_solver
from .iklib import tweaker as ik_tweaker

AlignMode = Literal["full", "lite"]


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


def _direction_angle_rad(actual_dir: np.ndarray, desired_dir: np.ndarray) -> float:
    dot = float(np.clip(np.dot(actual_dir, desired_dir), -1.0, 1.0))
    return float(math.acos(dot))


def _refine_orientation(
    *,
    q: np.ndarray,
    hold_target: np.ndarray,
    unit_dir: np.ndarray,
    context: dict,
    position_hold_tol_m: float,
    tweak_rounds: int,
    align_mode: AlignMode,
    timing: Optional["PickTimingCollector"],
) -> ik_aligner.OrientationRefineResult:
    refine_kwargs = dict(
        current_q=q,
        target_world=hold_target,
        target_dir_world=unit_dir,
        context=context,
        position_hold_tol_m=position_hold_tol_m,
    )
    if str(align_mode).strip().lower() == "lite":
        return ik_aligner.refine_direction_lite(**refine_kwargs)
    return ik_aligner.refine_direction_with_position_hold(
        **refine_kwargs,
        rounds=int(tweak_rounds),
    )


def solve_position_only(
    *,
    target_world: Sequence[float],
    target_dir_world: Optional[Sequence[float]],
    context: dict,
    position_tol_m: float,
    max_iters: int,
    current_seed: Sequence[float],
    timing: Optional["PickTimingCollector"] = None,
) -> SolveAndAlignResult:
    """Position IK only; direction angle measured from FK (no align refine)."""
    if timing is not None:
        with timing.span("solve_position"):
            result = ik_solver.solve_ik(
                target_world=target_world,
                context=context,
                position_tol_m=position_tol_m,
                max_iters=max_iters,
                current_seed=current_seed,
            )
    else:
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
    direction_angle_rad = 0.0
    initial_direction_angle_rad = 0.0
    if target_dir_world is not None:
        direction = np.asarray(target_dir_world, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(direction))
        if dnorm > 1e-9:
            unit_dir = direction / dnorm
            actual_dir = ik_kin._forward_grasp_direction_world(context, q)
            direction_angle_rad = _direction_angle_rad(actual_dir, unit_dir)
            initial_direction_angle_rad = float(direction_angle_rad)

    return SolveAndAlignResult(
        success=True,
        q=q,
        position_error_m=err_m,
        seed_name=str(result.seed_name),
        iterations=int(result.iterations),
        direction_angle_rad=float(direction_angle_rad),
        initial_direction_angle_rad=float(initial_direction_angle_rad),
        reason="position_only",
    )


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
    align_mode: AlignMode = "full",
    align_skip_under_deg: Optional[float] = None,
    timing: Optional["PickTimingCollector"] = None,
) -> SolveAndAlignResult:
    if timing is not None:
        with timing.span("solve_position"):
            result = ik_solver.solve_ik(
                target_world=target_world,
                context=context,
                position_tol_m=position_tol_m,
                max_iters=max_iters,
                current_seed=current_seed,
            )
    else:
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
    align_skipped = False
    align_position_kept = False
    align_direction_improved = False
    direction_angle_rad = 0.0
    initial_direction_angle_rad = 0.0
    if target_dir_world is not None:
        direction = np.asarray(target_dir_world, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(direction))
        if dnorm > 1e-9:
            unit_dir = direction / dnorm
            actual_dir = ik_kin._forward_grasp_direction_world(context, q)
            initial_direction_angle_rad = _direction_angle_rad(actual_dir, unit_dir)
            skip_under_rad = (
                math.radians(float(align_skip_under_deg))
                if align_skip_under_deg is not None
                else None
            )
            if skip_under_rad is not None and float(initial_direction_angle_rad) <= float(skip_under_rad):
                direction_angle_rad = float(initial_direction_angle_rad)
                align_attempted = False
                align_skipped = True
            else:
                align_attempted = True
                hold_target = ik_kin._forward_grasp_world(context, q)
                if timing is not None:
                    with timing.span("align_direction"):
                        refine = _refine_orientation(
                            q=q,
                            hold_target=hold_target,
                            unit_dir=unit_dir,
                            context=context,
                            position_hold_tol_m=tweak_position_hold_tol_m,
                            tweak_rounds=tweak_rounds,
                            align_mode=align_mode,
                            timing=timing,
                        )
                else:
                    refine = _refine_orientation(
                        q=q,
                        hold_target=hold_target,
                        unit_dir=unit_dir,
                        context=context,
                        position_hold_tol_m=tweak_position_hold_tol_m,
                        tweak_rounds=tweak_rounds,
                        align_mode=align_mode,
                        timing=None,
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
    elif align_skipped:
        reason = "position_converged_align_skipped"

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


def _look_at_object_dir(
    position: Sequence[float],
    object_world: Sequence[float],
) -> Optional[np.ndarray]:
    obj = np.asarray(object_world, dtype=float).reshape(3)
    pos = np.asarray(position, dtype=float).reshape(3)
    vec = obj - pos
    norm = float(np.linalg.norm(vec))
    if norm <= 1e-9:
        return None
    return vec / norm


def solve_then_look_at_tweak(
    *,
    target_world: Sequence[float],
    object_world: Optional[Sequence[float]] = None,
    target_dir_world: Optional[Sequence[float]] = None,
    context: dict,
    position_tol_m: float,
    max_iters: int,
    current_seed: Sequence[float],
    tweak_position_hold_tol_m: float = 1.5e-2,
    tweak_rounds: int = 8,
    direction_tol_deg: float = 5.0,
    align_skip_under_deg: Optional[float] = None,
    timing: Optional["PickTimingCollector"] = None,
) -> SolveAndAlignResult:
    """Position IK then look-at-object tweak (no full align seed bank)."""
    if timing is not None:
        with timing.span("solve_position"):
            result = ik_solver.solve_ik(
                target_world=target_world,
                context=context,
                position_tol_m=position_tol_m,
                max_iters=max_iters,
                current_seed=current_seed,
            )
    else:
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
    align_skipped = False
    direction_angle_rad = 0.0
    initial_direction_angle_rad = 0.0

    obj_arr = None if object_world is None else np.asarray(object_world, dtype=float).reshape(3)
    if obj_arr is not None:
        pos_now = ik_kin._forward_grasp_world(context, q)
        actual_dir = ik_kin._forward_grasp_direction_world(context, q)
        look_dir = _look_at_object_dir(pos_now, obj_arr)
        if look_dir is not None:
            initial_direction_angle_rad = _direction_angle_rad(actual_dir, look_dir)
            direction_angle_rad = float(initial_direction_angle_rad)
            skip_under_rad = (
                math.radians(float(align_skip_under_deg))
                if align_skip_under_deg is not None
                else None
            )
            if skip_under_rad is not None and float(initial_direction_angle_rad) <= float(skip_under_rad):
                align_skipped = True
            else:
                align_attempted = True
                target_pos = np.asarray(target_world, dtype=float).reshape(3)
                if timing is not None:
                    with timing.span("tweak_look_at"):
                        tweak = ik_tweaker.tweak_pose_look_at_object(
                            current_q=q,
                            target_world=target_pos,
                            object_world=obj_arr,
                            context=context,
                            position_tol_m=tweak_position_hold_tol_m,
                            direction_tol_deg=direction_tol_deg,
                            max_iters=int(tweak_rounds),
                        )
                else:
                    tweak = ik_tweaker.tweak_pose_look_at_object(
                        current_q=q,
                        target_world=target_pos,
                        object_world=obj_arr,
                        context=context,
                        position_tol_m=tweak_position_hold_tol_m,
                        direction_tol_deg=direction_tol_deg,
                        max_iters=int(tweak_rounds),
                    )
                q = np.asarray(tweak.q, dtype=float).reshape(4).copy()
                err_m = float(tweak.position_error_m)
                direction_angle_rad = float(tweak.direction_angle_rad)
                pos_final = ik_kin._forward_grasp_world(context, q)
                look_final = _look_at_object_dir(pos_final, obj_arr)
                if look_final is not None:
                    dir_final = ik_kin._forward_grasp_direction_world(context, q)
                    direction_angle_rad = _direction_angle_rad(dir_final, look_final)
    elif target_dir_world is not None:
        direction = np.asarray(target_dir_world, dtype=float).reshape(3)
        dnorm = float(np.linalg.norm(direction))
        if dnorm > 1e-9:
            unit_dir = direction / dnorm
            actual_dir = ik_kin._forward_grasp_direction_world(context, q)
            initial_direction_angle_rad = _direction_angle_rad(actual_dir, unit_dir)
            direction_angle_rad = float(initial_direction_angle_rad)
            skip_under_rad = (
                math.radians(float(align_skip_under_deg))
                if align_skip_under_deg is not None
                else None
            )
            if skip_under_rad is not None and float(initial_direction_angle_rad) <= float(skip_under_rad):
                align_skipped = True
            else:
                align_attempted = True
                target_pos = np.asarray(target_world, dtype=float).reshape(3)
                if timing is not None:
                    with timing.span("tweak_look_at"):
                        tweak = ik_tweaker.tweak_pose(
                            current_q=q,
                            target_world=target_pos,
                            target_dir_world=unit_dir,
                            context=context,
                            position_tol_m=tweak_position_hold_tol_m,
                            direction_tol_deg=direction_tol_deg,
                            max_iters=int(tweak_rounds),
                        )
                else:
                    tweak = ik_tweaker.tweak_pose(
                        current_q=q,
                        target_world=target_pos,
                        target_dir_world=unit_dir,
                        context=context,
                        position_tol_m=tweak_position_hold_tol_m,
                        direction_tol_deg=direction_tol_deg,
                        max_iters=int(tweak_rounds),
                    )
                q = np.asarray(tweak.q, dtype=float).reshape(4).copy()
                err_m = float(tweak.position_error_m)
                direction_angle_rad = float(tweak.direction_angle_rad)

    reason = "position_converged"
    if align_attempted:
        reason = "position_converged_tweak_look_at"
    elif align_skipped:
        reason = "position_converged_tweak_skipped"

    return SolveAndAlignResult(
        success=True,
        q=q,
        position_error_m=err_m,
        seed_name=str(result.seed_name),
        iterations=int(result.iterations),
        align_attempted=align_attempted,
        align_position_kept=True,
        align_direction_improved=align_attempted,
        direction_angle_rad=float(direction_angle_rad),
        initial_direction_angle_rad=float(initial_direction_angle_rad),
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
    align_mode: AlignMode = "full",
    align_skip_under_deg: Optional[float] = None,
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
        align_mode=align_mode,
        align_skip_under_deg=align_skip_under_deg,
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


def begin_tweak_session(
    *,
    current_q: Sequence[float],
    hold_target_world: Sequence[float],
    target_dir_world: Sequence[float],
    initial_step_scale: float = 1.0,
):
    return ik_tweaker.begin_tweak_session(
        current_q=current_q,
        target_world=hold_target_world,
        target_dir_world=target_dir_world,
        initial_step_scale=initial_step_scale,
    )


def evaluate_tweak_feedback(
    *,
    session,
    actual_tip_world: Sequence[float],
    actual_dir_world: Sequence[float],
    position_tol_m: float = 5e-3,
    direction_tol_deg: float = 5.0,
    stable_success_required: int = 2,
):
    return ik_tweaker.evaluate_tweak_feedback(
        session=session,
        actual_tip_world=actual_tip_world,
        actual_dir_world=actual_dir_world,
        position_tol_m=position_tol_m,
        direction_tol_deg=direction_tol_deg,
        stable_success_required=stable_success_required,
    )


def accept_tweak_step(*, session, step):
    return ik_tweaker.accept_tweak_step(session=session, step=step)


def reject_tweak_step(*, session, step=None):
    return ik_tweaker.reject_tweak_step(session=session, step=step)


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


def compute_tweak_session_step(
    *,
    session,
    context: dict,
    actual_tip_world: Sequence[float],
    actual_dir_world: Sequence[float],
):
    return ik_tweaker.compute_tweak_session_step(
        session=session,
        context=context,
        actual_tip_world=actual_tip_world,
        actual_dir_world=actual_dir_world,
    )


__all__ = [
    "AlignMode",
    "SolveAndAlignResult",
    "accept_tweak_step",
    "begin_tweak_session",
    "compute_tweak_session_step",
    "load_solver_context",
    "evaluate_tweak_feedback",
    "reject_tweak_step",
    "solve_position_only",
    "solve_then_align",
    "solve_then_look_at_tweak",
    "solve_then_tweak",
    "compute_tweak_step",
    "tweak_only",
]


# Backward-compatibility alias for older callers.
SolveAndTweakResult = SolveAndAlignResult
