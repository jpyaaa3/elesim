from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np
from scipy.optimize import least_squares

from .kinematics import Q_BENT, Q_NEUTRAL, _ReachModel


@dataclass(frozen=True)
class OrientationRefineResult:
    q: np.ndarray
    position_error_m: float
    direction_error: float
    direction_angle_rad: float
    initial_direction_error: float
    initial_direction_angle_rad: float
    iterations: int
    accepted_steps: int
    position_kept: bool
    direction_improved: bool
    converged: bool


def _solve_position_only_trf(
    model: _ReachModel,
    target_world: np.ndarray,
    q0: np.ndarray,
) -> tuple[np.ndarray, float]:
    bounds_lo = np.array([model.linear_min, model.roll_min, -model.bend_lim, -model.bend_lim], dtype=float)
    bounds_hi = np.array([model.linear_max, model.roll_max, +model.bend_lim, +model.bend_lim], dtype=float)

    def residual(q: np.ndarray) -> np.ndarray:
        return np.asarray(model.error_vec(q, target_world), dtype=float).reshape(3)

    result = least_squares(
        residual,
        x0=np.asarray(model.clamp_q(q0), dtype=float),
        bounds=(bounds_lo, bounds_hi),
        method="trf",
        loss="linear",
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
        max_nfev=200,
    )
    q_sol = model.clamp_q(result.x)
    err = float(np.linalg.norm(model.error_vec(q_sol, target_world)))
    return q_sol, err


def _solve_position_with_fixed_roll_trf(
    model: _ReachModel,
    target_world: np.ndarray,
    q0: np.ndarray,
    *,
    fixed_roll_rad: float,
) -> tuple[np.ndarray, float]:
    q_seed = np.asarray(model.clamp_q(q0), dtype=float).reshape(4).copy()
    q_seed[1] = float(np.clip(fixed_roll_rad, model.roll_min, model.roll_max))
    bounds_lo = np.array([model.linear_min, -model.bend_lim, -model.bend_lim], dtype=float)
    bounds_hi = np.array([model.linear_max, +model.bend_lim, +model.bend_lim], dtype=float)

    def residual(x: np.ndarray) -> np.ndarray:
        q = np.array([x[0], fixed_roll_rad, x[1], x[2]], dtype=float)
        return np.asarray(model.error_vec(q, target_world), dtype=float).reshape(3)

    x0 = np.array([q_seed[0], q_seed[2], q_seed[3]], dtype=float)
    result = least_squares(
        residual,
        x0=x0,
        bounds=(bounds_lo, bounds_hi),
        method="trf",
        loss="linear",
        ftol=1e-10,
        xtol=1e-10,
        gtol=1e-10,
        max_nfev=200,
    )
    q_sol = model.clamp_q(np.array([result.x[0], fixed_roll_rad, result.x[1], result.x[2]], dtype=float))
    err = float(np.linalg.norm(model.error_vec(q_sol, target_world)))
    return q_sol, err


def _generate_align_seed_bank(
    model: _ReachModel,
    *,
    current_q: Sequence[float],
    random_count: int,
    rng: np.random.Generator,
) -> list[np.ndarray]:
    bend = float(min(model.bend_lim, math.radians(36.0)))
    q_cur = model.clamp_q(current_q)
    linear_mid = 0.5 * (model.linear_min + model.linear_max)
    roll_bank = [0.0, math.radians(45.0), -math.radians(45.0), math.radians(90.0), -math.radians(90.0), float(q_cur[1])]
    linear_bank = [model.linear_min, linear_mid, model.linear_max, float(q_cur[0])]
    bend_bank = [
        (0.0, 0.0),
        (-bend, +bend),
        (+bend, -bend),
        (+bend, +bend),
        (-bend, -bend),
        (float(q_cur[2]), float(q_cur[3])),
    ]

    seeds = [
        np.asarray(Q_NEUTRAL, dtype=float).reshape(4).copy(),
        np.asarray(Q_BENT, dtype=float).reshape(4).copy(),
        np.array([0.0, 0.0, +bend, -bend], dtype=float),
        np.array([0.0, 0.0, -bend, -bend], dtype=float),
        np.array([0.0, 0.0, +bend, +bend], dtype=float),
        q_cur.copy(),
    ]
    for linear in linear_bank:
        for roll in roll_bank:
            for theta1, theta2 in bend_bank:
                seeds.append(np.array([linear, roll, theta1, theta2], dtype=float))
    for _ in range(max(int(random_count), 0)):
        seeds.append(
            np.array(
                [
                    rng.uniform(model.linear_min, model.linear_max),
                    rng.uniform(model.roll_min, model.roll_max),
                    rng.uniform(-model.bend_lim, model.bend_lim),
                    rng.uniform(-model.bend_lim, model.bend_lim),
                ],
                dtype=float,
            )
        )

    out: list[np.ndarray] = []
    seen: set[tuple[float, ...]] = set()
    for seed in seeds:
        q = model.clamp_q(seed)
        key = tuple(np.round(q, 6))
        if key in seen:
            continue
        seen.add(key)
        out.append(q)
    return out


def _normalize_dir(vec: Sequence[float]) -> Optional[np.ndarray]:
    arr = np.asarray(vec, dtype=float).reshape(3)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-9:
        return None
    return arr / norm


def _direction_angle_rad(actual_dir: np.ndarray, desired_dir: np.ndarray) -> float:
    dot = float(np.clip(np.dot(actual_dir, desired_dir), -1.0, 1.0))
    return float(np.arccos(dot))


def refine_direction_with_position_hold(
    *,
    current_q: Sequence[float],
    target_world: Sequence[float],
    target_dir_world: Sequence[float],
    context: dict,
    position_hold_tol_m: float = 1.5e-2,
    rounds: int = 12,
) -> OrientationRefineResult:
    model = _ReachModel(context=context, limit=context["limit"])
    q0 = model.clamp_q(current_q)
    target_world_np = np.asarray(target_world, dtype=float).reshape(3)
    target_dir_np = _normalize_dir(target_dir_world)
    if target_dir_np is None:
        pos_err = float(np.linalg.norm(model.error_vec(q0, target_world_np)))
        return OrientationRefineResult(
            q=q0.copy(),
            position_error_m=pos_err,
            direction_error=0.0,
            direction_angle_rad=0.0,
            initial_direction_error=0.0,
            initial_direction_angle_rad=0.0,
            iterations=0,
            accepted_steps=0,
            position_kept=(pos_err <= float(max(position_hold_tol_m, 1e-6))),
            direction_improved=False,
            converged=(pos_err <= float(max(position_hold_tol_m, 1e-6))),
        )

    pos_tol = float(max(position_hold_tol_m, 1e-6))
    rng = np.random.default_rng(0)
    seed_bank = _generate_align_seed_bank(
        model,
        current_q=q0,
        random_count=max(16, int(rounds) * 4),
        rng=rng,
    )
    total_iters = 0

    phase1_best_q = q0.copy()
    phase1_best_err = float(np.linalg.norm(model.error_vec(phase1_best_q, target_world_np)))
    for q_seed in seed_bank:
        q_sol, err = _solve_position_only_trf(model, target_world_np, q_seed)
        total_iters += 1
        if err < phase1_best_err:
            phase1_best_q = q_sol.copy()
            phase1_best_err = float(err)
        if err <= pos_tol:
            phase1_best_q = q_sol.copy()
            phase1_best_err = float(err)
            break

    fixed_roll_rad = float(phase1_best_q[1])
    best_q = phase1_best_q.copy()
    best_pos = float(phase1_best_err)
    start_dir = float(model.direction_error(best_q, target_dir_np))
    start_dir_angle = _direction_angle_rad(model.grasp_direction(best_q), target_dir_np)
    best_dir = float(start_dir)

    seen_sol: set[tuple[float, ...]] = set()
    for q_seed in seed_bank:
        q_seed_fixed = np.asarray(q_seed, dtype=float).reshape(4).copy()
        q_seed_fixed[1] = fixed_roll_rad
        q_sol, err = _solve_position_with_fixed_roll_trf(
            model,
            target_world_np,
            q_seed_fixed,
            fixed_roll_rad=fixed_roll_rad,
        )
        total_iters += 1
        if err > pos_tol:
            continue
        sol_key = tuple(np.round(np.asarray([q_sol[0], q_sol[2], q_sol[3]], dtype=float), 4))
        if sol_key in seen_sol:
            continue
        seen_sol.add(sol_key)
        dir_err = float(model.direction_error(q_sol, target_dir_np))
        if (dir_err + 1e-10) < best_dir or (abs(dir_err - best_dir) <= 1e-10 and err < best_pos):
            best_q = q_sol.copy()
            best_pos = float(err)
            best_dir = dir_err

    best_dir_angle = _direction_angle_rad(model.grasp_direction(best_q), target_dir_np)
    position_kept = bool(best_pos <= pos_tol)
    direction_improved = bool(best_dir + 1e-10 < start_dir)
    return OrientationRefineResult(
        q=best_q.copy(),
        position_error_m=float(best_pos),
        direction_error=float(best_dir),
        direction_angle_rad=float(best_dir_angle),
        initial_direction_error=float(start_dir),
        initial_direction_angle_rad=float(start_dir_angle),
        iterations=int(total_iters),
        accepted_steps=0,
        position_kept=position_kept,
        direction_improved=direction_improved,
        converged=(position_kept and direction_improved),
    )


__all__ = [
    "OrientationRefineResult",
    "refine_direction_with_position_hold",
]
