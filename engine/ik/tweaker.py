from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from .kinematics import _ReachModel, _damped_pinv, _limited_step


@dataclass(frozen=True)
class OrientationRefineResult:
    q: np.ndarray
    position_error_m: float
    direction_error: float
    direction_angle_rad: float
    iterations: int
    accepted_steps: int
    converged: bool


def _normalize_dir(vec: Sequence[float]) -> Optional[np.ndarray]:
    arr = np.asarray(vec, dtype=float).reshape(3)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-9:
        return None
    return arr / norm


def _direction_angle_rad(actual_dir: np.ndarray, desired_dir: np.ndarray) -> float:
    dot = float(np.clip(np.dot(actual_dir, desired_dir), -1.0, 1.0))
    return float(np.arccos(dot))


def _pose_cost(
    *,
    pos_err_m: float,
    dir_angle_rad: float,
    pos_tol_m: float,
    dir_tol_rad: float,
    direction_weight: float,
) -> float:
    pos_scale = float(max(pos_tol_m, 1e-3))
    dir_scale = float(max(dir_tol_rad, math.radians(3.0)))
    return float((pos_err_m / pos_scale) ** 2 + float(max(direction_weight, 0.0)) * (dir_angle_rad / dir_scale) ** 2)


def _task_priority_step(
    *,
    model: _ReachModel,
    q: np.ndarray,
    target_world: np.ndarray,
    target_dir: np.ndarray,
    damping: float,
    direction_gain: float,
) -> np.ndarray:
    pos_now = model.grasp_position(q)
    dir_now = model.grasp_direction(q)
    e_p = np.asarray(target_world, dtype=float).reshape(3) - np.asarray(pos_now, dtype=float).reshape(3)
    e_d = direction_gain * (np.asarray(target_dir, dtype=float).reshape(3) - np.asarray(dir_now, dtype=float).reshape(3))

    J_p = model.position_jacobian(q)
    J_d = model.direction_jacobian(q)

    J_p_pinv = _damped_pinv(J_p, damping=damping)
    dq_p = J_p_pinv @ e_p

    N_p = np.eye(4, dtype=float) - J_p_pinv @ J_p
    J_d_null = J_d @ N_p
    rhs_d = e_d - J_d @ dq_p
    z = _damped_pinv(J_d_null, damping=damping) @ rhs_d

    dq = np.asarray(dq_p + N_p @ z, dtype=float).reshape(4)
    return _limited_step(
        dq,
        max_linear_m=0.008,
        max_angle_rad=math.radians(3.0),
    )


def refine_direction_with_position_hold(
    *,
    current_q: Sequence[float],
    target_world: Sequence[float],
    target_dir_world: Sequence[float],
    context: dict,
    position_hold_tol_m: float = 1.5e-2,
    rounds: int = 10,
    direction_tol_deg: float = 6.0,
    direction_weight: float = 0.35,
    direction_gain: float = 1.0,
    damping: float = 1e-2,
) -> OrientationRefineResult:
    model = _ReachModel(context=context, limit=context["limit"])
    q = model.clamp_q(current_q)
    target_world_np = np.asarray(target_world, dtype=float).reshape(3)
    target_dir_np = _normalize_dir(target_dir_world)
    if target_dir_np is None:
        pos_err = float(np.linalg.norm(model.error_vec(q, target_world_np)))
        return OrientationRefineResult(
            q=q.copy(),
            position_error_m=pos_err,
            direction_error=0.0,
            direction_angle_rad=0.0,
            iterations=0,
            accepted_steps=0,
            converged=(pos_err <= float(max(position_hold_tol_m, 1e-6))),
        )

    pos_tol = float(max(position_hold_tol_m, 1e-6))
    dir_tol_rad = math.radians(float(max(direction_tol_deg, 0.1)))

    pos_now = model.grasp_position(q)
    dir_now = model.grasp_direction(q)
    best_pos_err = float(np.linalg.norm(target_world_np - pos_now))
    best_dir_angle = _direction_angle_rad(dir_now, target_dir_np)
    best_dir_error = float(1.0 - np.clip(float(np.dot(dir_now, target_dir_np)), -1.0, 1.0))
    best_cost = _pose_cost(
        pos_err_m=best_pos_err,
        dir_angle_rad=best_dir_angle,
        pos_tol_m=pos_tol,
        dir_tol_rad=dir_tol_rad,
        direction_weight=direction_weight,
    )
    best_q = q.copy()

    accepted_steps = 0
    step_scale = 1.0
    min_step_scale = 1.0 / 64.0
    max_iters = max(int(rounds), 1)

    for _iter in range(max_iters):
        if best_pos_err <= pos_tol and best_dir_angle <= dir_tol_rad:
            return OrientationRefineResult(
                q=best_q.copy(),
                position_error_m=float(best_pos_err),
                direction_error=float(best_dir_error),
                direction_angle_rad=float(best_dir_angle),
                iterations=_iter,
                accepted_steps=accepted_steps,
                converged=True,
            )

        raw_step = _task_priority_step(
            model=model,
            q=q,
            target_world=target_world_np,
            target_dir=target_dir_np,
            damping=damping,
            direction_gain=direction_gain,
        )
        if float(np.linalg.norm(raw_step)) <= 1e-10:
            break

        accepted = False
        trial_scale = step_scale
        while trial_scale >= min_step_scale:
            q_trial = model.clamp_q(q + float(trial_scale) * raw_step)
            if float(np.linalg.norm(q_trial - q)) <= 1e-10:
                trial_scale *= 0.5
                continue

            pos_trial = model.grasp_position(q_trial)
            dir_trial = model.grasp_direction(q_trial)
            pos_err_trial = float(np.linalg.norm(target_world_np - pos_trial))
            dir_angle_trial = _direction_angle_rad(dir_trial, target_dir_np)
            dir_error_trial = float(1.0 - np.clip(float(np.dot(dir_trial, target_dir_np)), -1.0, 1.0))
            cost_trial = _pose_cost(
                pos_err_m=pos_err_trial,
                dir_angle_rad=dir_angle_trial,
                pos_tol_m=pos_tol,
                dir_tol_rad=dir_tol_rad,
                direction_weight=direction_weight,
            )

            if cost_trial + 1e-12 < best_cost:
                q = q_trial
                best_q = q_trial.copy()
                best_pos_err = pos_err_trial
                best_dir_angle = dir_angle_trial
                best_dir_error = dir_error_trial
                best_cost = cost_trial
                accepted_steps += 1
                step_scale = min(1.0, trial_scale * 1.15)
                accepted = True
                break

            trial_scale *= 0.5

        if not accepted:
            step_scale *= 0.5
            if step_scale < min_step_scale:
                break

    return OrientationRefineResult(
        q=best_q.copy(),
        position_error_m=float(best_pos_err),
        direction_error=float(best_dir_error),
        direction_angle_rad=float(best_dir_angle),
        iterations=max_iters,
        accepted_steps=accepted_steps,
        converged=(best_pos_err <= pos_tol and best_dir_angle <= dir_tol_rad),
    )


__all__ = [
    "OrientationRefineResult",
    "refine_direction_with_position_hold",
]
