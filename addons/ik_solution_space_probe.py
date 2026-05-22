#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import csv
import math
import sys
import tkinter as tk
from dataclasses import dataclass
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Iterable, Optional

import numpy as np
from scipy.optimize import least_squares

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from engine.ik import kinematics as core_kin
from engine.ik import solver as core_solver


@dataclass(frozen=True)
class ProbeSolution:
    q: np.ndarray
    position_error_m: float
    direction_world: np.ndarray


@dataclass(frozen=True)
class ProbeResult:
    solutions: list[ProbeSolution]
    target_world: np.ndarray
    tolerance_m: float
    raw_solution_count: int
    unique_solution_count: int
    fixed_roll_rad: float
    csv_path: Path
    plot_path: Optional[Path]


def _deg(rad: float) -> float:
    return float(np.degrees(float(rad)))


def _make_seed_bank(model: core_kin._ReachModel, *, random_count: int, rng: np.random.Generator) -> list[np.ndarray]:
    bend = float(min(model.bend_lim, math.radians(36.0)))
    linear_mid = 0.5 * (model.linear_min + model.linear_max)
    roll_bank = [0.0, math.radians(45.0), -math.radians(45.0), math.radians(90.0), -math.radians(90.0)]
    linear_bank = [model.linear_min, linear_mid, model.linear_max]
    bend_bank = [
        (0.0, 0.0),
        (-bend, +bend),
        (+bend, -bend),
        (+bend, +bend),
        (-bend, -bend),
    ]

    seeds = [
        np.asarray(core_kin.Q_NEUTRAL, dtype=float).reshape(4).copy(),
        np.asarray(core_kin.Q_BENT, dtype=float).reshape(4).copy(),
        np.array([0.0, 0.0, +bend, -bend], dtype=float),
        np.array([0.0, 0.0, -bend, -bend], dtype=float),
        np.array([0.0, 0.0, +bend, +bend], dtype=float),
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
    return [model.clamp_q(seed) for seed in seeds]


def _solve_position_only(
    model: core_kin._ReachModel,
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


def _solve_position_with_fixed_roll(
    model: core_kin._ReachModel,
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


def _dedupe_solutions(solutions: Iterable[ProbeSolution]) -> list[ProbeSolution]:
    out: list[ProbeSolution] = []
    seen: set[tuple[float, ...]] = set()
    for sol in sorted(solutions, key=lambda s: s.position_error_m):
        q = np.asarray(sol.q, dtype=float).reshape(4)
        key = tuple(np.round(np.array([q[0], q[1], q[2], q[3]], dtype=float), 4))
        if key in seen:
            continue
        seen.add(key)
        out.append(sol)
    return out


def _write_csv(path: Path, solutions: list[ProbeSolution]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "linear_m",
                "roll_deg",
                "theta1_deg",
                "theta2_deg",
                "pos_err_mm",
                "dir_x",
                "dir_y",
                "dir_z",
            ]
        )
        for sol in solutions:
            q = np.asarray(sol.q, dtype=float).reshape(4)
            d = np.asarray(sol.direction_world, dtype=float).reshape(3)
            writer.writerow(
                [
                    float(q[0]),
                    _deg(q[1]),
                    _deg(q[2]),
                    _deg(q[3]),
                    float(sol.position_error_m * 1000.0),
                    float(d[0]),
                    float(d[1]),
                    float(d[2]),
                ]
            )


def _write_plot(path: Path, solutions: list[ProbeSolution], target_world: np.ndarray, fixed_roll_rad: float) -> Optional[Path]:
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except Exception:
        return None

    path.parent.mkdir(parents=True, exist_ok=True)
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")

    if solutions:
        q_mat = np.stack([np.asarray(sol.q, dtype=float).reshape(4) for sol in solutions], axis=0)
        ax.scatter(
            q_mat[:, 0],
            np.degrees(q_mat[:, 2]),
            np.degrees(q_mat[:, 3]),
            s=24,
            alpha=0.9,
            color="#2a6fdb",
        )

    ax.set_xlabel("linear [m]")
    ax.set_ylabel("theta1 [deg]")
    ax.set_zlabel("theta2 [deg]")
    ax.set_title(
        "Position-valid IK solutions with fixed roll\n"
        f"target=({target_world[0]:.3f}, {target_world[1]:.3f}, {target_world[2]:.3f}) | "
        f"roll={_deg(fixed_roll_rad):.1f} deg"
    )
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def run_probe(
    *,
    config_path: str,
    target_world: np.ndarray,
    tolerance_m: float,
    random_count: int,
    random_seed: int,
    out_prefix: str,
) -> ProbeResult:
    _bundle, ctx = core_solver.load_solver_context(config_path)
    model = core_kin._ReachModel(context=ctx, limit=ctx["limit"])
    rng = np.random.default_rng(int(random_seed))
    phase1_best_q: Optional[np.ndarray] = None
    phase1_best_err = float("inf")
    for q0 in _make_seed_bank(model, random_count=random_count, rng=rng):
        q_sol, err = _solve_position_only(model, target_world, q0)
        if err < phase1_best_err:
            phase1_best_q = q_sol.copy()
            phase1_best_err = float(err)
        if err <= float(tolerance_m):
            phase1_best_q = q_sol.copy()
            phase1_best_err = float(err)
            break

    if phase1_best_q is None:
        phase1_best_q = np.asarray(core_ik.Q_NEUTRAL, dtype=float).reshape(4).copy()
    fixed_roll_rad = float(phase1_best_q[1])

    raw_solutions: list[ProbeSolution] = []
    for q0 in _make_seed_bank(model, random_count=random_count, rng=rng):
        q_seed = np.asarray(q0, dtype=float).reshape(4).copy()
        q_seed[1] = fixed_roll_rad
        q_sol, err = _solve_position_with_fixed_roll(
            model,
            target_world,
            q_seed,
            fixed_roll_rad=fixed_roll_rad,
        )
        if err > float(tolerance_m):
            continue
        raw_solutions.append(
            ProbeSolution(
                q=q_sol.copy(),
                position_error_m=float(err),
                direction_world=np.asarray(model.grasp_direction(q_sol), dtype=float).reshape(3),
            )
        )

    solutions = _dedupe_solutions(raw_solutions)
    prefix = Path(out_prefix)
    csv_path = prefix.with_suffix(".csv")
    png_path = prefix.with_suffix(".png")
    _write_csv(csv_path, solutions)
    plot_path = _write_plot(png_path, solutions, target_world, fixed_roll_rad)
    return ProbeResult(
        solutions=solutions,
        target_world=np.asarray(target_world, dtype=float).reshape(3),
        tolerance_m=float(tolerance_m),
        raw_solution_count=len(raw_solutions),
        unique_solution_count=len(solutions),
        fixed_roll_rad=fixed_roll_rad,
        csv_path=csv_path,
        plot_path=plot_path,
    )


class ProbeApp:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("IK Solution Space Probe")

        self.config_var = tk.StringVar(value="config.ini")
        self.tx_var = tk.StringVar(value="0.180")
        self.ty_var = tk.StringVar(value="0.000")
        self.tz_var = tk.StringVar(value="0.100")
        self.tol_var = tk.StringVar(value="3.0")
        self.random_var = tk.StringVar(value="192")
        self.seed_var = tk.StringVar(value="0")
        self.out_var = tk.StringVar(value="addons/output/ik_probe")
        self.reference_var = tk.StringVar(value="Reference: loading from config.ini ...")

        frame = ttk.Frame(root, padding=12)
        frame.grid(row=0, column=0, sticky="nsew")
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)

        fields = [
            ("Config", self.config_var),
            ("Target X [m]", self.tx_var),
            ("Target Y [m]", self.ty_var),
            ("Target Z [m]", self.tz_var),
            ("Tol [mm]", self.tol_var),
            ("Random seeds", self.random_var),
            ("Random seed", self.seed_var),
            ("Out prefix", self.out_var),
        ]
        for row, (label, var) in enumerate(fields):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(frame, textvariable=var, width=36).grid(row=row, column=1, sticky="ew", pady=2)

        frame.columnconfigure(1, weight=1)

        ttk.Label(
            frame,
            textvariable=self.reference_var,
            justify="left",
            wraplength=700,
        ).grid(row=len(fields), column=0, columnspan=2, sticky="w", pady=(8, 4))

        ttk.Button(frame, text="Reload Reference", command=self.refresh_reference).grid(
            row=len(fields) + 1,
            column=0,
            sticky="ew",
            pady=(4, 8),
        )
        ttk.Button(frame, text="Run Probe", command=self.on_run).grid(
            row=len(fields) + 1,
            column=1,
            sticky="ew",
            pady=(4, 8),
        )

        self.output = tk.Text(frame, width=84, height=18, wrap="word")
        self.output.grid(row=len(fields) + 2, column=0, columnspan=2, sticky="nsew")
        frame.rowconfigure(len(fields) + 2, weight=1)

        self.refresh_reference()

    def refresh_reference(self) -> None:
        config_path = self.config_var.get().strip() or "config.ini"
        try:
            _bundle, ctx = core_solver.load_solver_context(config_path)
            spawn = np.asarray(ctx["spawn_xyz"], dtype=float).reshape(3)
            root_link = str(ctx["fk_root_link"])
            root_local = np.asarray(ctx["part_pose_root"].get(root_link, np.zeros(3, dtype=float)), dtype=float).reshape(3)
            root_world = spawn + root_local
            linear_joint = str(ctx["linear_joint_name"])
            self.reference_var.set(
                "Reference: target input is in world coordinates. "
                f"Robot root link '{root_link}' starts at world "
                f"({root_world[0]:.3f}, {root_world[1]:.3f}, {root_world[2]:.3f}). "
                f"Config spawn_position = ({spawn[0]:.3f}, {spawn[1]:.3f}, {spawn[2]:.3f}). "
                f"Linear control joint = '{linear_joint}'."
            )
        except Exception as exc:
            self.reference_var.set(f"Reference: failed to load config/robot structure: {exc}")

    def log(self, msg: str) -> None:
        self.output.insert("end", msg + "\n")
        self.output.see("end")
        self.root.update_idletasks()

    def on_run(self) -> None:
        try:
            config_path = self.config_var.get().strip() or "config.ini"
            target_world = np.array(
                [
                    float(self.tx_var.get()),
                    float(self.ty_var.get()),
                    float(self.tz_var.get()),
                ],
                dtype=float,
            )
            tolerance_m = float(self.tol_var.get()) * 1e-3
            random_count = int(self.random_var.get())
            random_seed = int(self.seed_var.get())
            out_prefix = self.out_var.get().strip() or "addons/output/ik_probe"
        except ValueError as exc:
            messagebox.showerror("Invalid input", str(exc))
            return

        self.output.delete("1.0", "end")
        self.log(
            "[probe] target=(%.4f, %.4f, %.4f) tol=%.2f mm"
            % (float(target_world[0]), float(target_world[1]), float(target_world[2]), float(tolerance_m) * 1000.0)
        )
        try:
            result = run_probe(
                config_path=config_path,
                target_world=target_world,
                tolerance_m=tolerance_m,
                random_count=random_count,
                random_seed=random_seed,
                out_prefix=out_prefix,
            )
        except Exception as exc:
            messagebox.showerror("Probe failed", str(exc))
            self.log(f"[probe] failed: {exc}")
            return

        self.log(
            "[probe] fixed roll = %.1f deg | raw_solutions=%d unique_solutions=%d"
            % (_deg(result.fixed_roll_rad), result.raw_solution_count, result.unique_solution_count)
        )
        if result.solutions:
            best = min(result.solutions, key=lambda s: s.position_error_m)
            q = np.asarray(best.q, dtype=float).reshape(4)
            self.log(
                "[probe] best err=%.3f mm | linear=%.4f m roll=%.1f deg theta1=%.1f deg theta2=%.1f deg"
                % (
                    float(best.position_error_m * 1000.0),
                    float(q[0]),
                    _deg(q[1]),
                    _deg(q[2]),
                    _deg(q[3]),
                )
            )
        else:
            self.log("[probe] no position-valid solutions found")

        self.log(f"[probe] csv: {result.csv_path}")
        if result.plot_path is not None:
            self.log(f"[probe] plot: {result.plot_path}")
        else:
            self.log("[probe] plot skipped: matplotlib unavailable")


def main() -> None:
    root = tk.Tk()
    root.geometry("780x520")
    ProbeApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
