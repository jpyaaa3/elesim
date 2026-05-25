from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .kinematics import _ReachModel, _damped_pinv, _limited_step


@dataclass(frozen=True)
class TweakStepResult:
    q: np.ndarray
    position_error_m: float
    direction_angle_rad: float
    cost: float
    step_scale: float
    accepted: bool


@dataclass(frozen=True)
class TweakResult:
    q: np.ndarray
    position_error_m: float
    direction_angle_rad: float
    iterations: int
    accepted_steps: int
    converged: bool
    reason: str = ""


def _normalize_dir(vec: Sequence[float] | np.ndarray | None) -> Optional[np.ndarray]:
    if vec is None:
        return None
    arr = np.asarray(vec, dtype=float).reshape(3)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-9:
        return None
    return arr / norm


def _direction_error_vec(actual_dir: np.ndarray, desired_dir: np.ndarray) -> np.ndarray:
    return np.cross(np.asarray(actual_dir, dtype=float).reshape(3), np.asarray(desired_dir, dtype=float).reshape(3))


def _direction_angle_rad(actual_dir: np.ndarray, desired_dir: np.ndarray) -> float:
    dot = float(np.clip(np.dot(actual_dir, desired_dir), -1.0, 1.0))
    return float(np.arccos(dot))


def _pose_cost(
    pos_err: np.ndarray,
    actual_dir: np.ndarray,
    desired_dir: np.ndarray,
    *,
    position_weight: float,
    direction_weight: float,
) -> tuple[float, float]:
    pos_norm = float(np.linalg.norm(np.asarray(pos_err, dtype=float).reshape(3)))
    dir_angle = _direction_angle_rad(actual_dir, desired_dir)
    cost = float(max(position_weight, 0.0)) * (pos_norm ** 2) + float(max(direction_weight, 0.0)) * (dir_angle ** 2)
    return cost, dir_angle


def compute_tweak_step(
    *,
    current_q: Sequence[float],
    target_world: Sequence[float],
    target_dir_world: Sequence[float],
    context: dict,
    actual_tip_world: Optional[Sequence[float]] = None,
    actual_dir_world: Optional[Sequence[float]] = None,
    position_weight: float = 1.0,
    direction_weight: float = 0.35,
    damping: float = 1e-3,
    step_scale: float = 1.0,
    line_search_steps: int = 6,
) -> TweakStepResult:
    model = _ReachModel(context=context, limit=context["limit"])
    q = model.clamp_q(current_q)
    target_pos = np.asarray(target_world, dtype=float).reshape(3)
    desired_dir = _normalize_dir(target_dir_world)
    if desired_dir is None:
        pos_err = target_pos - model.grasp_position(q)
        return TweakStepResult(
            q=q.copy(),
            position_error_m=float(np.linalg.norm(pos_err)),
            direction_angle_rad=0.0,
            cost=float(np.linalg.norm(pos_err) ** 2),
            step_scale=0.0,
            accepted=False,
        )

    model_pos = model.grasp_position(q)
    model_dir = model.grasp_direction(q)
    actual_pos = model_pos if actual_tip_world is None else np.asarray(actual_tip_world, dtype=float).reshape(3)
    actual_dir = model_dir
    if actual_dir_world is not None:
        actual_dir_candidate = _normalize_dir(actual_dir_world)
        if actual_dir_candidate is not None:
            actual_dir = actual_dir_candidate

    pos_err = target_pos - actual_pos
    cost_old, dir_angle_old = _pose_cost(
        pos_err,
        actual_dir,
        desired_dir,
        position_weight=position_weight,
        direction_weight=direction_weight,
    )

    Jp = model.position_jacobian(q)
    Jd = model.direction_jacobian(q)
    Jp_pinv = _damped_pinv(Jp, damping=float(max(damping, 1e-9)))
    dq_pos = Jp_pinv @ pos_err
    Np = np.eye(4, dtype=float) - Jp_pinv @ Jp

    dir_err = _direction_error_vec(actual_dir, desired_dir)
    Jd_eff = Jd @ Np
    rhs_dir = dir_err - Jd @ dq_pos
    dq_dir = _damped_pinv(Jd_eff, damping=float(max(damping, 1e-9))) @ rhs_dir
    dq_full = dq_pos + float(max(direction_weight, 0.0)) * (Np @ dq_dir)

    best = TweakStepResult(
        q=q.copy(),
        position_error_m=float(np.linalg.norm(pos_err)),
        direction_angle_rad=float(dir_angle_old),
        cost=float(cost_old),
        step_scale=0.0,
        accepted=False,
    )

    base_scale = float(max(step_scale, 0.0))
    for ls_idx in range(max(int(line_search_steps), 1)):
        alpha = base_scale * (0.5 ** ls_idx)
        dq_try = _limited_step(alpha * dq_full)
        q_try = model.clamp_q(q + dq_try)
        try_pos = model.grasp_position(q_try)
        try_dir = model.grasp_direction(q_try)
        try_err = target_pos - try_pos
        cost_new, dir_angle_new = _pose_cost(
            try_err,
            try_dir,
            desired_dir,
            position_weight=position_weight,
            direction_weight=direction_weight,
        )
        if cost_new + 1e-12 < best.cost:
            best = TweakStepResult(
                q=q_try.copy(),
                position_error_m=float(np.linalg.norm(try_err)),
                direction_angle_rad=float(dir_angle_new),
                cost=float(cost_new),
                step_scale=float(alpha),
                accepted=True,
            )
            break

    return best


def tweak_pose(
    *,
    current_q: Sequence[float],
    target_world: Sequence[float],
    target_dir_world: Sequence[float],
    context: dict,
    actual_tip_world: Optional[Sequence[float]] = None,
    actual_dir_world: Optional[Sequence[float]] = None,
    position_tol_m: float = 5e-3,
    direction_tol_deg: float = 5.0,
    position_weight: float = 1.0,
    direction_weight: float = 0.35,
    damping: float = 1e-3,
    max_iters: int = 10,
    initial_step_scale: float = 1.0,
) -> TweakResult:
    model = _ReachModel(context=context, limit=context["limit"])
    q = model.clamp_q(current_q)
    target_pos = np.asarray(target_world, dtype=float).reshape(3)
    desired_dir = _normalize_dir(target_dir_world)
    if desired_dir is None:
        pos_err = float(np.linalg.norm(target_pos - model.grasp_position(q)))
        return TweakResult(
            q=q.copy(),
            position_error_m=pos_err,
            direction_angle_rad=0.0,
            iterations=0,
            accepted_steps=0,
            converged=(pos_err <= float(max(position_tol_m, 1e-6))),
            reason="invalid target direction",
        )

    pos_tol = float(max(position_tol_m, 1e-6))
    dir_tol = math.radians(float(max(direction_tol_deg, 0.1)))
    accepted_steps = 0
    step_scale = float(max(initial_step_scale, 1e-3))

    for iteration in range(1, max(int(max_iters), 1) + 1):
        pos_now = model.grasp_position(q)
        dir_now = model.grasp_direction(q)
        pos_err_now = target_pos - pos_now
        dir_ang_now = _direction_angle_rad(dir_now, desired_dir)
        if float(np.linalg.norm(pos_err_now)) <= pos_tol and float(dir_ang_now) <= dir_tol:
            return TweakResult(
                q=q.copy(),
                position_error_m=float(np.linalg.norm(pos_err_now)),
                direction_angle_rad=float(dir_ang_now),
                iterations=iteration - 1,
                accepted_steps=accepted_steps,
                converged=True,
                reason="converged",
            )

        step = compute_tweak_step(
            current_q=q,
            target_world=target_pos,
            target_dir_world=desired_dir,
            context=context,
            actual_tip_world=actual_tip_world,
            actual_dir_world=actual_dir_world,
            position_weight=position_weight,
            direction_weight=direction_weight,
            damping=damping,
            step_scale=step_scale,
        )
        if not step.accepted:
            return TweakResult(
                q=q.copy(),
                position_error_m=float(np.linalg.norm(pos_err_now)),
                direction_angle_rad=float(dir_ang_now),
                iterations=iteration,
                accepted_steps=accepted_steps,
                converged=False,
                reason="no improving step",
            )
        q = step.q.copy()
        accepted_steps += 1
        step_scale = max(step.step_scale * 0.75, 0.1)

    pos_final = model.grasp_position(q)
    dir_final = model.grasp_direction(q)
    pos_err_final = target_pos - pos_final
    dir_ang_final = _direction_angle_rad(dir_final, desired_dir)
    return TweakResult(
        q=q.copy(),
        position_error_m=float(np.linalg.norm(pos_err_final)),
        direction_angle_rad=float(dir_ang_final),
        iterations=max(int(max_iters), 1),
        accepted_steps=accepted_steps,
        converged=(float(np.linalg.norm(pos_err_final)) <= pos_tol and float(dir_ang_final) <= dir_tol),
        reason="iteration limit",
    )


__all__ = [
    "TweakResult",
    "TweakStepResult",
    "compute_tweak_step",
    "tweak_pose",
]
