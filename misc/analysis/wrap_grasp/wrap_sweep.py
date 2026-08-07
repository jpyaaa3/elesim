#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Wrap-grasp (theta1, theta2) feasibility sweep for the continuum arm.

Reuses the real forward kinematics (``kinematics._forward_link_tf``) and the
real collision-proxy distance functions (``planning.collision``:
``segment_segment_distance``, ``segment_box_distance``, ``self_collision_check``)
-- no distance math is reimplemented here. This module only adds the
higher-level search (grid + coarse-to-fine refinement for the largest
inscribed cylinder, and angular sampling for the wrap-angle/escape-gate
caging metrics), which the existing code has no equivalent of.

Object model: a cylinder of radius r and fixed axial length L, axis aligned
with world Y (the bending-plane normal when roll=0, since all bend joints
rotate about local Y and roll is held at 0). Its axis is represented as a
line segment and tested against every arm-link primitive using the arm's own
segment-vs-capsule / segment-vs-box distance functions -- the same
functions used for two arm links, just with one "link" replaced by the
candidate object's axis segment.

sag correction is disabled by construction: the loaded IK context has no
"sag_model" key (see extract_geometry.py's note), so
kinematics._build_q_map's per-joint sag error defaults to zero and nominal
(uncorrected) geometry is used. Shape/positioning uncertainty is instead
modeled via --shape-margin, which uniformly inflates every arm surface
(implemented as subtracting the margin from every raw clearance value --
equivalent to growing the arm's proxy geometry outward by that margin).

Run with:
    PYTHONPATH=packages/protocol/src:controller/src python3 \\
        misc/analysis/wrap_grasp/wrap_sweep.py --shape-margin 0.0 \\
        --out-csv wrap_feasibility.csv --out-strategy strategy_map.json
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from typing import Any, Optional, Sequence

import numpy as np

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
ARM_MODEL_PATH = os.path.join(REPO_ROOT, "controller", "config", "arm_model.json")
COLLISION_MODEL_PATH = os.path.join(REPO_ROOT, "controller", "config", "collision_model.json")

from elesim_controller.robot.arm.iklib import kinematics as ik_kin  # noqa: E402
from elesim_controller.robot.arm.joint_defs import JointLimit  # noqa: E402
from elesim_controller.robot.arm.planning import collision as ik_collision  # noqa: E402


def load_context() -> dict[str, Any]:
    with open(ARM_MODEL_PATH, "r", encoding="utf-8") as f:
        model = json.load(f)
    if int(model.get("schema_version", 0)) != 1:
        raise RuntimeError(f"unsupported arm model schema: {ARM_MODEL_PATH}")
    context = dict(model["context"])
    limit_raw = context["limit"]
    context["limit"] = JointLimit(**dict(limit_raw["value"]))
    assert "sag_model" not in context, "context unexpectedly carries a sag_model -- nominal-geometry assumption broken"
    return context


def load_collision_model() -> ik_collision.CollisionModel:
    return ik_collision.CollisionModel.from_json(COLLISION_MODEL_PATH)


def min_dist_segment_to_arm(
    seg_a: np.ndarray,
    seg_b: np.ndarray,
    link_tf: dict[str, tuple[np.ndarray, np.ndarray]],
    model: ik_collision.CollisionModel,
    margin: float,
) -> float:
    """Surface-to-surface gap from the candidate object's axis segment to the nearest arm link.

    Dispatches per-link to the arm's own capsule/box distance functions,
    exactly mirroring ``_link_pair_gap`` (collision.py) but with one side
    fixed to the object's axis instead of another arm link.
    """
    best = math.inf
    for name, (pos, rot) in link_tf.items():
        if model.is_box(name):
            for center, half, box_rot in model.world_boxes(name, pos, rot):
                d = ik_collision.segment_box_distance(seg_a, seg_b, center, half, box_rot)
                if d < best:
                    best = d
        else:
            p0, p1, radius = model.world_capsule(name, pos, rot)
            d = ik_collision.segment_segment_distance(seg_a, seg_b, p0, p1) - radius
            if d < best:
                best = d
    return best - margin


def _axis_segment(x: float, z: float, half_len: float) -> tuple[np.ndarray, np.ndarray]:
    return np.array([x, -half_len, z]), np.array([x, half_len, z])


def find_center_and_rmax(
    link_tf: dict[str, tuple[np.ndarray, np.ndarray]],
    model: ik_collision.CollisionModel,
    xz_bounds: tuple[float, float, float, float],
    half_len: float,
    margin: float,
    coarse_n: int,
    refine_passes: int,
    shrink: float,
) -> tuple[float, float, float]:
    """Coarse-to-fine grid search for the largest inscribed-cylinder center/radius.

    Not a global optimizer: it locates the best point within the searched
    window and refines around it. Whether that point is actually a secure
    cage (as opposed to an open pocket that happens to be locally maximal)
    is a separate question, answered by ``evaluate_cage`` downstream.
    """
    xmin, xmax, zmin, zmax = xz_bounds
    cx, cz = (xmin + xmax) / 2.0, (zmin + zmax) / 2.0
    half_x, half_z = (xmax - xmin) / 2.0, (zmax - zmin) / 2.0
    best_x, best_z, best_d = cx, cz, -math.inf

    for _pass in range(refine_passes + 1):
        # Clamped to the ORIGINAL outer bounds every pass, not just the shrinking
        # window -- otherwise a locally-ascending search (e.g. an open, barely-bent
        # pose with no true enclosing cage) walks its window center outward each
        # pass and "escapes" the intended search region entirely.
        xs = np.linspace(max(cx - half_x, xmin), min(cx + half_x, xmax), coarse_n)
        zs = np.linspace(max(cz - half_z, zmin), min(cz + half_z, zmax), coarse_n)
        pass_best_x, pass_best_z, pass_best_d = best_x, best_z, best_d
        for x in xs:
            for z in zs:
                seg_a, seg_b = _axis_segment(float(x), float(z), half_len)
                d = min_dist_segment_to_arm(seg_a, seg_b, link_tf, model, margin)
                if d > pass_best_d:
                    pass_best_d, pass_best_x, pass_best_z = d, float(x), float(z)
        best_x, best_z, best_d = pass_best_x, pass_best_z, pass_best_d
        cx, cz = best_x, best_z
        half_x *= shrink
        half_z *= shrink

    return best_x, best_z, best_d


def _largest_false_run(contacted: np.ndarray) -> int:
    """Longest run of ``False`` in a circular boolean array (wraparound-aware)."""
    n = len(contacted)
    if not contacted.any():
        return n
    if contacted.all():
        return 0
    first_true = int(np.argmax(contacted))
    rolled = np.roll(contacted, -first_true)
    max_run = 0
    cur = 0
    for v in rolled:
        if not v:
            cur += 1
            if cur > max_run:
                max_run = cur
        else:
            cur = 0
    return max_run


def evaluate_cage(
    center_xz: tuple[float, float],
    r: float,
    half_len: float,
    link_tf: dict[str, tuple[np.ndarray, np.ndarray]],
    model: ik_collision.CollisionModel,
    margin: float,
    contact_tol: float,
    n_samples: int,
) -> tuple[float, float]:
    """Wrap angle Phi (deg) and escape-gate width G (m) at object radius r, center (cx,cz).

    Phi: angular measure (in degrees) of directions where the object's own
    surface (radius r) sits within ``contact_tol`` of an arm surface.
    G: chord width, on the object's OWN surface circle, of the largest
    contiguous angular arc with no arm contact -- a proxy for the physical
    escape corridor width, not an arm-to-arm throat measurement (that would
    require locating the actual bounding arm surfaces at the gap edges,
    which this analysis does not attempt). Documented explicitly because it
    means G is only reliable as "< 2r" evidence for gaps under ~180 degrees;
    see the strategy-map notes.
    """
    cx, cz = center_xz
    contacted = np.zeros(n_samples, dtype=bool)
    for k in range(n_samples):
        phi = 2.0 * math.pi * k / n_samples
        x = cx + r * math.cos(phi)
        z = cz + r * math.sin(phi)
        seg_a, seg_b = _axis_segment(x, z, half_len)
        d = min_dist_segment_to_arm(seg_a, seg_b, link_tf, model, margin)
        contacted[k] = d <= contact_tol

    phi_deg = 360.0 * float(np.count_nonzero(contacted)) / n_samples
    gap_run = _largest_false_run(contacted)
    gap_angle_rad = 2.0 * math.pi * gap_run / n_samples
    g = 2.0 * r * math.sin(min(gap_angle_rad, math.pi) / 2.0) if gap_angle_rad <= math.pi else 2.0 * r
    return phi_deg, g


def run_sweep(args: argparse.Namespace) -> list[dict[str, Any]]:
    context = load_context()
    model = load_collision_model()
    fk_chain = context["fk_joint_chain"]
    bend_deg = float(context["limit"].bend_deg)
    half_len = args.axis_length_m / 2.0

    n_steps = int(round(2 * bend_deg / args.theta_step_deg))
    theta_vals_deg = np.linspace(-bend_deg, bend_deg, n_steps + 1)

    rows: list[dict[str, Any]] = []
    total = len(theta_vals_deg) ** 2
    done = 0
    for theta1_deg in theta_vals_deg:
        for theta2_deg in theta_vals_deg:
            done += 1
            q4 = [
                args.linear_m,
                math.radians(args.roll_deg),
                math.radians(float(theta1_deg)),
                math.radians(float(theta2_deg)),
            ]
            link_tf = ik_kin._forward_link_tf(context, q4)
            self_res = ik_collision.self_collision_check(
                link_tf, fk_joint_chain=fk_chain, model=model, clearance_m=0.0
            )
            row: dict[str, Any] = {
                "theta1_deg": float(theta1_deg),
                "theta2_deg": float(theta2_deg),
                "self_collision_ok": bool(self_res.ok),
                "self_collision_clearance_m": float(self_res.min_clearance_m),
                "self_collision_pair": f"{self_res.link_a}|{self_res.link_b}",
                "r_max_m": None,
                "center_x_m": None,
                "center_z_m": None,
                "phi_deg": None,
                "g_m": None,
                "secure_wrap": False,
                "caged": False,
            }
            if self_res.ok:
                xs = [float(pos[0]) for pos, _rot in link_tf.values()]
                zs = [float(pos[2]) for pos, _rot in link_tf.values()]
                bounds = (
                    min(xs) - args.bbox_pad_m,
                    max(xs) + args.bbox_pad_m,
                    min(zs) - args.bbox_pad_m,
                    max(zs) + args.bbox_pad_m,
                )
                cx, cz, rmax = find_center_and_rmax(
                    link_tf, model, bounds, half_len, args.shape_margin,
                    args.coarse_n, args.refine_passes, args.refine_shrink,
                )
                rmax = max(rmax, 0.0)
                if rmax > 0.0:
                    phi_deg, g = evaluate_cage(
                        (cx, cz), rmax, half_len, link_tf, model, args.shape_margin,
                        args.contact_tol_m, args.angular_samples,
                    )
                else:
                    phi_deg, g = 0.0, 0.0
                row.update(
                    {
                        "r_max_m": rmax,
                        "center_x_m": cx,
                        "center_z_m": cz,
                        "phi_deg": phi_deg,
                        "g_m": g,
                        "secure_wrap": phi_deg >= 180.0,
                        "caged": g < 2.0 * rmax,
                    }
                )
            rows.append(row)
        if args.progress:
            print(f"[wrap_sweep] theta1={theta1_deg:+.1f} done ({done}/{total} combos)", flush=True)
    return rows


def write_csv(rows: list[dict[str, Any]], path: str) -> None:
    fieldnames = [
        "theta1_deg", "theta2_deg", "r_max_mm", "center_x_mm", "center_z_mm",
        "phi_deg", "g_mm", "secure_wrap", "caged",
        "self_collision_ok", "self_collision_clearance_mm", "self_collision_pair",
    ]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            writer.writerow(
                {
                    "theta1_deg": round(r["theta1_deg"], 3),
                    "theta2_deg": round(r["theta2_deg"], 3),
                    "r_max_mm": round(r["r_max_m"] * 1000, 3) if r["r_max_m"] is not None else "",
                    "center_x_mm": round(r["center_x_m"] * 1000, 3) if r["center_x_m"] is not None else "",
                    "center_z_mm": round(r["center_z_m"] * 1000, 3) if r["center_z_m"] is not None else "",
                    "phi_deg": round(r["phi_deg"], 2) if r["phi_deg"] is not None else "",
                    "g_mm": round(r["g_m"] * 1000, 3) if r["g_m"] is not None else "",
                    "secure_wrap": r["secure_wrap"],
                    "caged": r["caged"],
                    "self_collision_ok": r["self_collision_ok"],
                    "self_collision_clearance_mm": round(r["self_collision_clearance_m"] * 1000, 3),
                    "self_collision_pair": r["self_collision_pair"],
                }
            )


def build_strategy_map(rows: list[dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
    context = load_context()
    model = load_collision_model()
    fk_chain = context["fk_joint_chain"]
    half_len = args.axis_length_m / 2.0

    feasible = [r for r in rows if r["self_collision_ok"] and r["r_max_m"] is not None and r["r_max_m"] > 0.0]
    diam_entries = []
    d_mm = args.d_min_mm
    while d_mm <= args.d_max_mm + 1e-9:
        r_needed = d_mm / 2000.0
        candidates = [r for r in feasible if r["r_max_m"] >= r_needed]
        if not candidates:
            diam_entries.append({"diameter_mm": round(d_mm, 3), "clears": False, "caged": False, "secure_wrap": False})
            d_mm += args.d_step_mm
            continue

        # r_max(q) alone only means "some point that far from the arm exists" --
        # for an open/near-straight pose that point can be far outside any real
        # cage (see find_center_and_rmax's docstring), and near-straight poses
        # are exactly the lowest-bend-effort ones, so ranking candidates by
        # minimal effort would preferentially re-check the LEAST promising
        # poses first. Instead rank by each candidate's own already-computed
        # Phi (at its own r_max(q)) descending -- a pose that already wraps
        # well at its best-fit radius is the one worth re-checking at r_needed
        # -- and only fall back to bend effort as a tie-break. Capped at
        # top-K (not all candidates, which can number in the hundreds) to
        # keep the re-evaluation cost bounded.
        candidates = sorted(
            candidates, key=lambda r: (-r["phi_deg"], r["theta1_deg"] ** 2 + r["theta2_deg"] ** 2)
        )
        top_k = candidates[: args.strategy_topk]

        evaluated = []
        for cand in top_k:
            q4 = [
                args.linear_m,
                math.radians(args.roll_deg),
                math.radians(cand["theta1_deg"]),
                math.radians(cand["theta2_deg"]),
            ]
            link_tf = ik_kin._forward_link_tf(context, q4)
            phi_deg, g = evaluate_cage(
                (cand["center_x_m"], cand["center_z_m"]), r_needed, half_len, link_tf, model,
                args.shape_margin, args.contact_tol_m, args.angular_samples,
            )
            evaluated.append(
                {
                    "theta1_deg": cand["theta1_deg"],
                    "theta2_deg": cand["theta2_deg"],
                    "phi_deg": phi_deg,
                    "g_mm": g * 1000.0,
                    "secure_wrap": phi_deg >= 180.0,
                    "caged": g < 2.0 * r_needed,
                }
            )

        caged_evals = [e for e in evaluated if e["caged"]]
        secure_evals = [e for e in evaluated if e["secure_wrap"]]
        if secure_evals:
            best = min(secure_evals, key=lambda e: e["theta1_deg"] ** 2 + e["theta2_deg"] ** 2)
        elif caged_evals:
            best = min(caged_evals, key=lambda e: e["theta1_deg"] ** 2 + e["theta2_deg"] ** 2)
        else:
            best = min(evaluated, key=lambda e: e["theta1_deg"] ** 2 + e["theta2_deg"] ** 2)

        diam_entries.append(
            {
                "diameter_mm": round(d_mm, 3),
                "clears": True,
                "theta1_deg": round(best["theta1_deg"], 2),
                "theta2_deg": round(best["theta2_deg"], 2),
                "phi_deg": round(best["phi_deg"], 2),
                "g_mm": round(best["g_mm"], 3),
                "secure_wrap": bool(best["secure_wrap"]),
                "caged": bool(best["caged"]),
                "selection_rule": (
                    f"best-of-top-{args.strategy_topk} (by min theta1^2+theta2^2) candidates with "
                    "r_max(q)>=r_needed, re-evaluated at r=r_needed; prefers secure_wrap, then caged, "
                    "then least bend effort"
                ),
            }
        )
        d_mm += args.d_step_mm

    clears_diams = [e["diameter_mm"] for e in diam_entries if e["clears"]]
    secure_diams = [e["diameter_mm"] for e in diam_entries if e.get("secure_wrap")]
    caged_diams = [e["diameter_mm"] for e in diam_entries if e.get("caged")]

    return {
        "params": {
            "theta_step_deg": args.theta_step_deg,
            "axis_length_m": args.axis_length_m,
            "shape_margin_m": args.shape_margin,
            "contact_tol_m": args.contact_tol_m,
            "angular_samples": args.angular_samples,
            "strategy_topk": args.strategy_topk,
            "diameter_sweep_mm": {"min": args.d_min_mm, "max": args.d_max_mm, "step": args.d_step_mm},
        },
        "diameter_classification_note": (
            "'clears' = some swept, self-collision-free pose has r_max(q) >= D/2 (pure geometric "
            "clearance for the largest inscribed cylinder found by the bounded grid search -- can "
            "include physically meaningless 'far from a barely-bent arm' points, see "
            "find_center_and_rmax's docstring). 'caged' = escape-gate G < D at that diameter for at "
            "least one of the top-K least-bend-effort clearing poses (re-evaluated at r=D/2, not at "
            "r_max(q)). 'secure_wrap' = wrap angle Phi >= 180deg, a strictly stronger condition. "
            "Only 'caged'/'secure_wrap' should be read as 'the arm can actually wrap-grasp this "
            "diameter'; 'clears' alone is not evidence of a real wrap."
        ),
        "clears_diameter_range_mm": [min(clears_diams), max(clears_diams)] if clears_diams else None,
        "caged_diameter_range_mm": [min(caged_diams), max(caged_diams)] if caged_diams else None,
        "secure_wrap_diameter_range_mm": [min(secure_diams), max(secure_diams)] if secure_diams else None,
        "min_wrappable_diameter_mm": min(caged_diams) if caged_diams else None,
        "max_wrappable_diameter_mm": max(caged_diams) if caged_diams else None,
        "gripper_boundary_note": (
            "No calibrated gripper max-opening-width exists in this repo (see "
            "geometry_report.json C_gripper.max_opening_width_m) -- the lower boundary "
            "between 'must use gripper' and 'wrap-grasp possible' cannot be located from "
            "real hardware data and is left unset here rather than guessed."
        ),
        "diameter_map": diam_entries,
    }


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--theta-step-deg", type=float, default=2.0, help="theta1/theta2 sweep step [deg]")
    ap.add_argument("--linear-m", type=float, default=0.0, help="fixed linear joint value [m]")
    ap.add_argument("--roll-deg", type=float, default=0.0, help="fixed roll joint value [deg]")
    ap.add_argument("--axis-length-m", type=float, default=0.08, help="object cylinder axial length [m]")
    ap.add_argument(
        "--shape-margin", type=float, default=0.0,
        help="uniform outward inflation of arm collision surfaces [m] (shape/positioning uncertainty)",
    )
    ap.add_argument("--contact-tol-m", type=float, default=0.003, help="wrap-angle contact tolerance [m]")
    ap.add_argument("--angular-samples", type=int, default=180, help="angular samples for Phi/G")
    ap.add_argument("--coarse-n", type=int, default=13, help="grid resolution per coarse-to-fine pass")
    ap.add_argument("--refine-passes", type=int, default=4, help="number of grid-refinement passes")
    ap.add_argument("--refine-shrink", type=float, default=0.35, help="window shrink factor per refine pass")
    ap.add_argument("--bbox-pad-m", type=float, default=0.015, help="padding around link-position bbox for the search window")
    ap.add_argument(
        "--strategy-topk", type=int, default=30,
        help="for each target diameter, re-evaluate Phi/G for the top-K lowest-bend-effort clearing poses",
    )
    ap.add_argument("--d-min-mm", type=float, default=20.0)
    ap.add_argument("--d-max-mm", type=float, default=250.0)
    ap.add_argument("--d-step-mm", type=float, default=5.0)
    ap.add_argument("--out-csv", default="wrap_feasibility.csv")
    ap.add_argument("--out-strategy", default="strategy_map.json")
    ap.add_argument("--progress", action="store_true", help="print per-theta1 progress")
    ap.add_argument(
        "--from-csv", default=None,
        help="skip the (expensive) sweep and rebuild only the strategy map from an existing wrap_feasibility.csv "
        "(e.g. after changing --strategy-topk/--d-* without changing the pose sweep itself)",
    )
    return ap


def _rows_from_csv(path: str) -> list[dict[str, Any]]:
    def _f(v: str) -> Optional[float]:
        return float(v) if v != "" else None

    rows = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            r_max_mm = _f(r["r_max_mm"])
            rows.append(
                {
                    "theta1_deg": float(r["theta1_deg"]),
                    "theta2_deg": float(r["theta2_deg"]),
                    "self_collision_ok": r["self_collision_ok"] == "True",
                    "self_collision_clearance_m": float(r["self_collision_clearance_mm"]) / 1000.0,
                    "self_collision_pair": r["self_collision_pair"],
                    "r_max_m": r_max_mm / 1000.0 if r_max_mm is not None else None,
                    "center_x_m": _f(r["center_x_mm"]) / 1000.0 if r["center_x_mm"] != "" else None,
                    "center_z_m": _f(r["center_z_mm"]) / 1000.0 if r["center_z_mm"] != "" else None,
                    "phi_deg": _f(r["phi_deg"]),
                    "g_m": _f(r["g_mm"]) / 1000.0 if r["g_mm"] != "" else None,
                    "secure_wrap": r["secure_wrap"] == "True",
                    "caged": r["caged"] == "True",
                }
            )
    return rows


def main() -> None:
    args = build_arg_parser().parse_args()
    if args.from_csv:
        rows = _rows_from_csv(args.from_csv)
        print(f"loaded {len(rows)} poses from {args.from_csv} (sweep skipped)")
    else:
        rows = run_sweep(args)
        write_csv(rows, args.out_csv)
        n_ok = sum(1 for r in rows if r["self_collision_ok"])
        print(f"wrote {args.out_csv} ({len(rows)} poses, {n_ok} self-collision-free)")
    strategy = build_strategy_map(rows, args)
    with open(args.out_strategy, "w", encoding="utf-8") as f:
        json.dump(strategy, f, indent=2, ensure_ascii=False)
    print(f"wrote {args.out_strategy}")


if __name__ == "__main__":
    main()
