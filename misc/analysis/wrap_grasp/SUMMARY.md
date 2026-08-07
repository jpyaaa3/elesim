# Wrap-grasp geometry analysis — summary

Scripts: `misc/analysis/wrap_grasp/{extract_geometry,wrap_sweep,sanity_check,plot_strategy_map}.py`
Data/figures: `misc/analysis/wrap_grasp/output/`
Re-run (needs the repo's own `numpy`/`scipy`, not otherwise installed in this checkout — see note at bottom):

```
PYTHONPATH=packages/protocol/src:controller/src python3 misc/analysis/wrap_grasp/extract_geometry.py \
    --out misc/analysis/wrap_grasp/output/geometry_report.json
PYTHONPATH=packages/protocol/src:controller/src python3 misc/analysis/wrap_grasp/wrap_sweep.py \
    --theta-step-deg 2 --shape-margin 0.0   --out-csv output/wrap_feasibility_margin0mm.csv   --out-strategy output/strategy_map_margin0mm.json
PYTHONPATH=packages/protocol/src:controller/src python3 misc/analysis/wrap_grasp/wrap_sweep.py \
    --theta-step-deg 2 --shape-margin 0.008 --out-csv output/wrap_feasibility_margin8mm.csv   --out-strategy output/strategy_map_margin8mm.json
python3 misc/analysis/wrap_grasp/sanity_check.py --geometry-report output/geometry_report.json \
    --strategy-map output/strategy_map_margin0mm.json --out output/sanity_check.json
python3 misc/analysis/wrap_grasp/plot_strategy_map.py --strategy-margin0 output/strategy_map_margin0mm.json \
    --strategy-margin8 output/strategy_map_margin8mm.json --sanity-check output/sanity_check.json --out-prefix output/strategy_map
```

## Headline numbers

| Quantity | Value | Source |
|---|---|---|
| Bend-joint pitch `h` | **50.0 mm, uniform** (9/9 gaps identical) | `arm_model.json` bend-joint pivot positions |
| Non-bending lead-in before first joint | 28.0 mm | wedge frame → `j_wedge_node0` pivot |
| `n_seg` | 5 (×2 segments = 10 bend joints total) | `context.n_seg` |
| Per-node bend limit `α_max` | 36° | `JointLimit.bend_deg` |
| Segment bend range | ±180° (= 5 × 36°) | derived |
| Node capsule radius (`d_inner`=`d_outer`) | 56.22 mm (isotropic) | `collision_model.json.link_capsules` |
| GO2 mount offset (deployed) | (0.10, 0.00, 0.07) m, spawn height 0.32 m | `config.yaml: robot.go2.spawn` |
| **Caged diameter range, margin=0** | **55–200 mm** | `strategy_map_margin0mm.json` |
| **Caged diameter range, margin=8 mm** | **40–185 mm** | `strategy_map_margin8mm.json` |
| Secure wrap (Φ≥180°) | **not reached anywhere in the sweep** — peak Φ = 172° at θ1=−22°, θ2=−24° | full-resolution CSV |
| Analytic (single-arc) min wrappable diameter | 41.4 mm | `sanity_check.json` |

## Deviations from the RA-L expectation table (flagged as requested)

| Item | Expected | Found | |
|---|---|---|---|
| Pitch `h` | 41.5 mm | **50.0 mm** uniform, plus a separate 28 mm non-bending stub before the first bend joint | **DIFFERENT** |
| `n_seg` | 5 | 5 | match |
| Per-node max angle | 36° | 36° | match |
| Segment bend range | ±180° | ±180° | match |
| Total arm length | 415 mm | **ambiguous** — wedge→node9 = 478 mm; node0→node9 = 450 mm; +51.4 mm more to gripper_base. No single number matches 415 mm cleanly; reported as separate labeled lengths instead of forcing one figure. | **DIFFERENT/AMBIGUOUS** |
| Tendon offset `w` | 26 mm | **not modeled at all** — `grep -rni tendon` across the whole repo returns zero hits. The capsule-fitting code (`chain_axis_capsule`) takes radius = max perpendicular vertex distance, a single isotropic scalar; there is no concave/convex split anywhere in the schema or the fitting code. | **NOT FOUND / NOT MODELED** |
| Gripper max opening | ~80 mm | **not found.** The real gripper is a boolean `claw_closed` open/close command, not a continuous width. The only numeric claw range in the repo is a synthetic FK joint limit (±0.02 m per claw, 0.04 m combined) used purely to place the claw meshes for collision/FK — the model-builder code and its docstring make clear this is not a measured spec. | **NOT FOUND** |
| Min wrappable diameter | ~70 mm | Numeric: **55 mm** (margin=0) / 40 mm (margin=8mm). Analytic single-arc: 41.4 mm. The numeric margin=0 value is the closest of the three to the paper's figure, though still ~20% below it. | **same order of magnitude, not an exact match** |

## Sanity check (analytic vs numeric)

- `R_min = h/(2·sin(α_max/2))` = 80.90 mm
- Concave inner envelope at `α_max`: `R_min·cos(α_max/2)` = 76.94 mm
- Analytic min wrappable radius: `76.94 − 56.22` = 20.72 mm → **diameter 41.4 mm**
- Numeric (margin=0) caged minimum: **55 mm** → ratio numeric/analytic = **1.33×**

Same order of magnitude — the pipeline is not off by an order of magnitude or inverted, which is what this check is for. The gap itself is expected and explainable: the analytic formula is a single segment forming an idealized constant-curvature arc with the object centered exactly at its center of curvature; the numeric sweep instead grid-searches the real two-segment arm with the base assembly present, gates on real self-collision, and requires an escape-gate margin (`G<2r`) rather than bare tangency — every one of those makes the numeric result more conservative (larger), which is exactly the direction the 1.33× gap goes.

## Margin comparison (0 mm vs 8 mm) — a genuine, non-obvious result

Per-pose, the mechanism is exactly as expected: `shape_margin` is subtracted uniformly from every raw clearance, so for a fixed `(θ1,θ2)` the optimal center `(cx,cz)` **does not move** and `r_max(q)` shifts down by **exactly** the margin (verified: peak `r_max` across the whole sweep went from 318.67 mm → 310.67 mm, a difference of exactly 8.00 mm).

But the **caged diameter range** does not shrink symmetrically:

- Upper bound shrinks as expected: 200 mm → 185 mm (less room for large objects when the arm is "bulkier").
- Lower bound *also* shrinks: 55 mm → 40 mm — i.e. **smaller objects become cage-able that weren't before**, even though every individual pose's capacity went down.

This is real, not a selection-heuristic artifact (checked at both `--strategy-topk 15` and `30` with the same result): `Φ`/`G` are re-evaluated **at the target radius** `r_needed`, not at `r_max(q)`. A margin-inflated arm surface closes the open escape-gate angles *for a smaller object sitting in the same pocket* faster than it shrinks that pocket's own peak capacity — so a smaller candidate can end up more thoroughly surrounded even though the arm's absolute best-case reach is worse. Reported as-is rather than forced into the more intuitive "margin uniformly shrinks everything" narrative, since that narrative is what the numbers actually contradict.

## Values not found (reported as `null` in `geometry_report.json`, not guessed)

1. **Tendon routing offset `w`** — not modeled anywhere in the collision pipeline (isotropic capsules only; zero repo-wide grep hits for "tendon").
2. **`d_inner` vs `d_outer` split** — same root cause; capsules are single-radius by construction, so both equal the capsule radius.
3. **Gripper max opening width** — no calibrated spec exists; only a boolean open/close control plus an explicitly-flagged-as-non-spec 0.04 m FK placeholder. The strategy map's lower "must-use-gripper" boundary is therefore left unset rather than drawn from that placeholder.
4. **A single "total arm length" figure** — the concept is ambiguous in this model (lead-in stub + 10×50mm nodes + fixed gripper offset + grasp offset, each a legitimately different "length"); reported as separate labeled lengths.
5. Camera hand-eye extrinsic (`hand_eye.camera.json`, relative to `node9`) and the FK chain's own fixed `camera` joint (relative to `gripper_base`) are two independent representations that this analysis did not attempt to reconcile — flagged, not merged.

## Caveats on the numeric method (read before trusting a single number off the map)

- `r_max(q)` search is a **bounded** coarse-to-fine grid search (window = swept-link bounding box + 15 mm pad), not a literal infinite-plane search — documented in `find_center_and_rmax`'s docstring. A large `r_max` for a barely-bent pose can be a real but physically meaningless "far from a sparse arm" point; this is exactly why the strategy map gates on `caged`/`secure_wrap` (Φ/G) rather than on `r_max` alone — the `clears` column in `strategy_map.json` should never be read as evidence of an actual wrap by itself.
- The diameter→pose reverse map only re-evaluates the top-30 lowest-bend-effort clearing poses per diameter (bounded cost, not exhaustive over ~1000+ poses); the resulting `caged`/`secure_wrap` flags are a lower bound on what's achievable, not a certified optimum.
- Escape-gate `G` is computed as the chord width on the **object's own surface circle** for the largest uncontacted angular arc — a documented proxy for the physical escape corridor, not an arm-to-arm throat measurement. Reliable as "<2r ⇒ caged" evidence mainly for gaps under ~180°; see `evaluate_cage`'s docstring.
- No diameter in 20–250 mm reached secure wrap (Φ≥180°); the closest raw pose found was Φ=172° at θ1=−22°, θ2=−24° (both same sign, i.e. a genuine one-sided "C" curl — the opposite-sign `Q_BENT` constant seen in `kinematics.py` is not a good wrap posture by this metric).
- The caged/not-caged pattern gets patchy above ~150 mm (small isolated True cells alternating with False) — likely a real effect of few available poses near the sweep's capacity ceiling rather than a bug, but treat the upper boundary as fuzzier than the lower one.

## Environment note

This checkout has no `numpy`/`scipy` installed anywhere on `PATH` (`controller`'s own `pyproject.toml` lists them as dependencies, but no `.venv` exists in this working tree). All runs above used a throwaway venv in the session scratchpad (`pip install "numpy>=1.26,<2" "scipy>=1.11" matplotlib`) — nothing was added to the repo itself.
