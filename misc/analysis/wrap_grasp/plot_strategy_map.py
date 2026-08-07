#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Render the wrap-grasp strategy map: feasibility zones vs object diameter.

Three stacked panels sharing one x-axis (object diameter, mm) -- never a
dual-axis chart:
  1. Strategy strip for shape_margin=0
  2. Strategy strip for shape_margin=8mm
  3. Required per-node bend angle vs diameter: the analytic single-arc
     formula (sanity_check.py) alongside the actual numeric sweep's chosen
     pose (max(|theta1|,|theta2|) at each diameter, margin=0)

Colors follow the dataviz skill's validated reference palette (status pair
for the two-state strip; categorical slots 1/2 for the two bend-angle curves).

Run with:
    python3 misc/analysis/wrap_grasp/plot_strategy_map.py \\
        --strategy-margin0 output/strategy_map_margin0mm.json \\
        --strategy-margin8 output/strategy_map_margin8mm.json \\
        --sanity-check output/sanity_check.json \\
        --out-prefix output/strategy_map
"""

from __future__ import annotations

import argparse
import json
import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

COLOR_CAGED = "#0ca30c"       # status: good
COLOR_NOT_CAGED = "#9ec5f4"   # sequential blue, light step (neutral "clears only")
COLOR_UNKNOWN = "#c3c2b7"     # text-secondary-ish neutral, hatched
COLOR_CURVE_ANALYTIC = "#2a78d6"  # categorical slot 1 (blue)
COLOR_CURVE_NUMERIC = "#eb6834"   # categorical slot 2 (orange)
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID_COLOR = "#e3e2dd"


def _load(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _draw_strip(ax, strategy: dict, d_min: float, title: str) -> None:
    entries = strategy["diameter_map"]
    step = strategy["params"]["diameter_sweep_mm"]["step"]
    if d_min > entries[0]["diameter_mm"]:
        ax.broken_barh(
            [(0, entries[0]["diameter_mm"] - 0)], (0, 1),
            facecolors=COLOR_UNKNOWN, hatch="//", edgecolor="white", linewidth=0.5,
        )
    for e in entries:
        left = e["diameter_mm"] - step / 2.0
        color = COLOR_CAGED if e.get("caged") else COLOR_NOT_CAGED
        ax.broken_barh([(left, step)], (0, 1), facecolors=color, edgecolor="white", linewidth=0.3)

    ax.set_xlim(0, 250)
    ax.set_ylim(0, 1)
    ax.set_yticks([])
    ax.set_title(title, fontsize=10, color=TEXT_PRIMARY, loc="left", pad=4)
    for spine in ax.spines.values():
        spine.set_visible(False)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--strategy-margin0", required=True)
    ap.add_argument("--strategy-margin8", required=True)
    ap.add_argument("--sanity-check", required=True)
    ap.add_argument("--out-prefix", default="strategy_map")
    args = ap.parse_args()

    strat0 = _load(args.strategy_margin0)
    strat8 = _load(args.strategy_margin8)
    sanity = _load(args.sanity_check)
    d_min = strat0["params"]["diameter_sweep_mm"]["min"]

    fig, (ax_strip0, ax_strip8, ax_curve) = plt.subplots(
        3, 1, figsize=(10, 9.2), height_ratios=[1, 1, 2.6], sharex=True,
    )
    fig.patch.set_facecolor("white")

    _draw_strip(ax_strip0, strat0, d_min, "Wrap-grasp strategy — shape_margin = 0 mm (nominal geometry)")
    _draw_strip(ax_strip8, strat8, d_min, "Wrap-grasp strategy — shape_margin = 8 mm (measured raw sag error)")

    legend_handles = [
        Patch(facecolor=COLOR_UNKNOWN, hatch="//", edgecolor="white", label=f"< {d_min:.0f} mm: not swept / gripper opening unknown"),
        Patch(facecolor=COLOR_NOT_CAGED, label="clears geometrically, escape gate G ≥ D (not caged)"),
        Patch(facecolor=COLOR_CAGED, label="caged: G < D (secure wrap Φ≥180° not reached anywhere)"),
    ]
    fig.legend(
        handles=legend_handles, loc="upper center", bbox_to_anchor=(0.5, 1.0),
        ncol=1, fontsize=8.5, frameon=False, labelcolor=TEXT_SECONDARY,
    )

    r0 = strat0.get("caged_diameter_range_mm")
    r8 = strat8.get("caged_diameter_range_mm")
    ann_y = -0.55
    if r0:
        ax_strip0.annotate(
            f"caged range: {r0[0]:.0f}–{r0[1]:.0f} mm\n(min ≈ R_min·cos(α_max/2) − node-capsule radius, see sanity check)",
            xy=((r0[0] + r0[1]) / 2, 0), xytext=((r0[0] + r0[1]) / 2, ann_y),
            fontsize=7.5, color=TEXT_SECONDARY, ha="center", va="top",
            arrowprops=dict(arrowstyle="-", color=GRID_COLOR, lw=0.8),
        )
    if r8:
        ax_strip8.annotate(
            f"caged range: {r8[0]:.0f}–{r8[1]:.0f} mm",
            xy=((r8[0] + r8[1]) / 2, 0), xytext=((r8[0] + r8[1]) / 2, ann_y),
            fontsize=7.5, color=TEXT_SECONDARY, ha="center", va="top",
            arrowprops=dict(arrowstyle="-", color=GRID_COLOR, lw=0.8),
        )

    # -- bend-angle auxiliary curve --
    analytic_map = sanity["analytic"]["alpha_required_deg_by_radius_mm"]
    radii = sorted(float(r) for r in analytic_map.keys())
    diam_analytic = [2 * r for r in radii]
    # keys were written as python floats via json -> string keys; match robustly by value
    alpha_analytic = []
    for r in radii:
        for k, v in analytic_map.items():
            if abs(float(k) - r) < 1e-6:
                alpha_analytic.append(v)
                break

    numeric_diam = []
    numeric_alpha = []
    for e in strat0["diameter_map"]:
        if e.get("clears") and e.get("theta1_deg") is not None:
            numeric_diam.append(e["diameter_mm"])
            numeric_alpha.append(max(abs(e["theta1_deg"]), abs(e["theta2_deg"])))

    ax_curve.plot(
        diam_analytic, alpha_analytic, color=COLOR_CURVE_ANALYTIC, lw=2,
        label="analytic single-arc: α(r) = 2·asin(h / 2(r+d_inner))",
    )
    ax_curve.plot(
        numeric_diam, numeric_alpha, color=COLOR_CURVE_NUMERIC, lw=1.5, marker="o", markersize=3,
        linestyle="none", alpha=0.85,
        label="numeric sweep (margin=0): max(|θ1|,|θ2|) of chosen pose",
    )
    ax_curve.axhline(36.0, color=TEXT_SECONDARY, lw=0.8, linestyle="--")
    ax_curve.text(2, 37.2, "per-joint limit 36°", fontsize=7.5, color=TEXT_SECONDARY, ha="left")

    ax_curve.set_xlim(0, 250)
    ax_curve.set_ylim(0, 45)
    ax_curve.set_xlabel("Object diameter D [mm]", fontsize=9.5, color=TEXT_PRIMARY)
    ax_curve.set_ylabel("Required per-node bend angle [deg]", fontsize=9.5, color=TEXT_PRIMARY)
    ax_curve.grid(True, color=GRID_COLOR, lw=0.6)
    ax_curve.set_axisbelow(True)
    for spine in ("top", "right"):
        ax_curve.spines[spine].set_visible(False)
    ax_curve.legend(fontsize=8, frameon=False, loc="upper right")
    ax_curve.tick_params(colors=TEXT_SECONDARY, labelsize=8)

    footnote = (
        "Object: 80mm-long cylinder, axis ⊥ bending plane. Arm: h=50mm pitch (uniform, not the\n"
        "expected 41.5mm), α_max=36°/node, n_seg=5, node-capsule radius=56.2mm (isotropic; no\n"
        "tendon-offset data exists in this repo). Secure wrap (Φ≥180°) not reached anywhere in\n"
        "the sweep (peak Φ=172°, at θ1=-22°,θ2=-24°).  Source: misc/analysis/wrap_grasp/"
    )
    fig.text(0.01, 0.012, footnote, fontsize=6.5, color=TEXT_SECONDARY, ha="left", va="bottom")

    fig.tight_layout(rect=(0, 0.075, 1, 0.94))
    fig.savefig(f"{args.out_prefix}.png", dpi=200, facecolor="white")
    fig.savefig(f"{args.out_prefix}.svg", facecolor="white")
    print(f"wrote {args.out_prefix}.png and .svg")


if __name__ == "__main__":
    main()
