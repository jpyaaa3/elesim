"""Where can this arm actually cage a cylinder?

Before training can mean anything, two facts have to be established from the
built scene rather than assumed:

1. the object pose in ``object.pos_xyz`` must lie somewhere the arm can wrap,
2. the wrap angle the 4-DoF arm can reach at all must be known, so the success
   gate is set from geometry instead of hope.

This sweeps the per-node bend angles (theta1, theta2) across their limits, one
environment per pose, reads the resulting arm-link positions out of Genesis, and
for each pose finds the cylinder centre that maximises wrap coverage.  It is the
Genesis-side counterpart of the analytic sweep in ``misc/analysis/wrap_grasp``
on the ``motion_planning`` branch, which reported a peak of Phi = 172 deg and
never reached 180 deg.

Run::

    python -m elesim_sim.rl.workspace_probe --grid 9 --out sim/benchmarks/workspace.md
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import torch

from .configs.loader import load_config, WrapGraspConfig
from .envs.contacts import ContactClassifier
from .envs.coverage import CoverageMeter
from .scene import WrapGraspScene

_REPO_ROOT = Path(__file__).resolve().parents[4]


@dataclass
class PoseResult:
    linear: float
    roll: float
    theta1: float
    theta2: float
    phi_rad: float
    centre: tuple[float, float, float]
    n_near: int
    caged: bool = False
    #: Arm links in real physical contact with the object, from get_contacts.
    #: Geometric proximity is not contact: with radial_band_m at 0.09 a link
    #: 9 cm clear of the surface still counts towards Phi, so a high wrap angle
    #: can describe an arm curled near the object without touching it.
    n_contacts: int = 0
    collides: bool = False



def _drive_to_pose(
    scene, mapper, poses: torch.Tensor, dofs: list[int], *, ramp: int = 80, settle: int = 40
) -> None:
    """Move the arm from home to each commanded pose, gradually.

    Teleporting with ``set_dofs_position`` is what the first version did, and it
    does not work.  For poses whose target puts links inside the support or the
    quadruped, the constraint solver starts from a deep penetration and
    diverges: "Invalid constraint forces causing 'nan'".  Reducing the timestep,
    adding solver substeps and cutting the actuator force limit all fail to fix
    it, because the penetration is in the initial state rather than in the
    forces.

    Ramping the *command* leaves the solver a physical path, and it also makes
    the measurement mean the right thing: a pose the arm cannot reach without
    colliding is simply not reached, instead of being scored from a
    configuration it teleported into.
    """
    home = torch.zeros_like(poses)
    for i in range(ramp):
        alpha = float(i + 1) / ramp
        targets = mapper.joint_targets(home + (poses - home) * alpha)
        scene.robot.control_dofs_position(targets, dofs_idx_local=dofs)
        scene.step()
    final = mapper.joint_targets(poses)
    for _ in range(settle):
        scene.robot.control_dofs_position(final, dofs_idx_local=dofs)
        scene.step()

def _candidate_centres(
    link_pos: torch.Tensor,
    radius: float,
    *,
    ring: int = 12,
    radial_steps: int = 5,
    floor_z: Optional[float] = None,
) -> torch.Tensor:
    """Candidate cylinder centres per pose.

    The arm wraps *around* something, so the centre worth testing sits inside
    the curve the links trace.  Offsetting the link centroid outward along
    several directions and distances covers that without an optimiser.

    When `floor_z` is given, every candidate is snapped to that height: a
    cylinder that has to be picked up off the ground cannot be floating, and
    leaving the height free lets the search find poses that score well around a
    mid-air object nobody can place.
    """
    centroid = link_pos.mean(dim=1)                       # (n, 3)
    device = link_pos.device
    offsets = [torch.zeros(3, device=device)]
    for k in range(ring):
        angle = 2.0 * math.pi * k / ring
        direction = torch.tensor(
            [math.cos(angle), math.sin(angle), 0.0], device=device
        )
        for step in range(1, radial_steps + 1):
            offsets.append(direction * (radius * step * 0.6))
    grid = torch.stack(offsets, dim=0)                    # (C, 3)
    centres = centroid.unsqueeze(1) + grid.unsqueeze(0)   # (n, C, 3)
    if floor_z is not None:
        centres = centres.clone()
        centres[..., 2] = float(floor_z)
    return centres


def probe(
    cfg: WrapGraspConfig,
    *,
    grid: int,
    roll_steps: int = 9,
    linear_steps: int = 4,
    free_height: bool = False,
    at_config_centre: bool = False,
    rank_by_contact: bool = False,
) -> tuple[list[PoseResult], dict]:
    """Sweep all four commanded DoFs.

    Sweeping only (theta1, theta2) is not enough and gets the answer wrong.
    Roll rotates the whole bend plane and is the only way this arm points
    downward, so with roll pinned at zero the arm curls sideways at mount
    height and can never reach an object resting on the floor.  The linear
    stage sets how far out the curl starts.  All four therefore have to move.
    """
    bend_lim = float(cfg.arm.limits.bend_per_node_rad)
    lin_lo, lin_hi = cfg.arm.limits.linear_m
    roll_lo, roll_hi = cfg.arm.limits.roll_rad
    bend_axis = torch.linspace(-bend_lim, bend_lim, grid)
    roll_axis = torch.linspace(float(roll_lo), float(roll_hi), roll_steps)
    lin_axis = torch.linspace(float(lin_lo), float(lin_hi), linear_steps)
    lin, roll, t1, t2 = torch.meshgrid(
        lin_axis, roll_axis, bend_axis, bend_axis, indexing="ij"
    )
    poses = torch.stack(
        (lin.reshape(-1), roll.reshape(-1), t1.reshape(-1), t2.reshape(-1)), dim=-1
    )
    n_envs = poses.shape[0]

    scene = WrapGraspScene(cfg, n_envs=n_envs).build()
    device = scene.device
    poses = poses.to(device)

    from .arm_kinematics import ArmWaypointMapper

    rate = cfg.macro_step.rate_limit
    mapper = ArmWaypointMapper(
        cfg.arm,
        n_envs=n_envs,
        device=device,
        rate_limit=(rate.linear_m, rate.roll_rad, rate.theta_rad, rate.theta_rad),
    )
    dofs = list(scene.arm_dofs.all_indices)
    _drive_to_pose(scene, mapper, poses.to(device), dofs)

    arm_ids = torch.tensor(sorted(scene.links.arm_local), device=device, dtype=torch.long)
    link_pos = scene.robot.get_links_pos()[:, arm_ids, :]

    snap = ContactClassifier(scene).classify(n_envs)
    pose_collides = snap.floor_touch | snap.go2_touch | snap.self_touch
    pose_contacts = snap.object_link_hits.sum(dim=-1)

    radius = float(cfg.object.radius_m)
    height = float(cfg.object.height_m)
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    # The object sits where the config actually resets it -- on the support
    # when one is configured, on the ground otherwise.  Snapping candidates to
    # any other height measures a placement the environment never produces.
    # `free_height` lifts the restriction entirely, which is what the analytic
    # sweep in misc/analysis/wrap_grasp effectively did: best centre anywhere in
    # the bend plane, nothing underneath.  Comparing the two isolates how much
    # of the wrap is blocked by whatever the object is standing on.
    floor_z = None if free_height else float(cfg.object_center()[2])
    if at_config_centre:
        # The best hypothetical centre answers "where should the object go";
        # this answers "what can the arm do with the object where it actually
        # is", which is the ceiling training will run into.
        fixed = torch.tensor(
            [float(v) for v in cfg.object_center()], device=device, dtype=torch.float32
        )
        centres = fixed.view(1, 1, 3).expand(n_envs, 1, 3)
    else:
        centres = _candidate_centres(link_pos, radius, floor_z=floor_z)
    upright = torch.tensor([1.0, 0.0, 0.0, 0.0], device=device).expand(n_envs, 4)
    link_r = float(cfg.reward.coverage.link_radius_m)
    tol = float(cfg.reward.coverage.interpenetration_tol_m)

    best_phi = torch.full((n_envs,), -1.0, device=device)
    best_centre = torch.zeros((n_envs, 3), device=device)
    best_near = torch.zeros((n_envs,), device=device, dtype=torch.long)
    best_caged = torch.zeros((n_envs,), device=device, dtype=torch.bool)
    r_vec = torch.full((n_envs,), radius, device=device)
    h_vec = torch.full((n_envs,), height, device=device)

    for c in range(centres.shape[1]):
        res = meter.measure(
            link_pos,
            centres[:, c, :],
            upright,
            radius_m=r_vec,
            height_m=h_vec,
            link_radius_m=link_r,
        )
        # A centre that puts a link inside the object is not a grasp, it is a
        # measurement artefact.  Reject it outright rather than letting it set
        # the peak wrap angle.
        feasible = res.min_surface_dist >= -tol
        better = feasible & (res.phi_rad > best_phi)
        best_caged = torch.where(better, res.caged, best_caged)
        best_phi = torch.where(better, res.phi_rad, best_phi)
        best_centre = torch.where(better.unsqueeze(-1), centres[:, c, :], best_centre)
        best_near = torch.where(better, res.n_near_links, best_near)

    results = [
        PoseResult(
            linear=float(poses[i, 0]),
            roll=float(poses[i, 1]),
            theta1=float(poses[i, 2]),
            theta2=float(poses[i, 3]),
            phi_rad=float(best_phi[i]),
            centre=tuple(float(v) for v in best_centre[i]),
            n_near=int(best_near[i]),
            caged=bool(best_caged[i]),
            n_contacts=int(pose_contacts[i]),
            collides=bool(pose_collides[i]),
        )
        for i in range(n_envs)
    ]
    results.sort(
        key=lambda r: (r.n_contacts if rank_by_contact else 0, r.phi_rad),
        reverse=True,
    )
    meta = {
        "link_radius_m": link_r,
        "interpenetration_tol_m": tol,
        "object_centre_z": None if floor_z is None else round(float(floor_z), 4),
        "free_height": bool(free_height),
        "at_config_centre": bool(at_config_centre),
        "rank_by_contact": bool(rank_by_contact),
        "n_poses_touching_object": int((pose_contacts > 0).sum()),
        "grid": grid,
        "roll_steps": roll_steps,
        "linear_steps": linear_steps,
        "n_poses": n_envs,
        "bend_per_node_rad": bend_lim,
        "object_radius_m": radius,
        "object_height_m": height,
        "coverage_bins": int(cfg.reward.coverage.n_bins),
        "radial_band_m": float(cfg.reward.coverage.radial_band_m),
        "scene": scene.describe(),
    }
    return results, meta



def placement_search(
    cfg: WrapGraspConfig, *, grid: int, roll_steps: int, linear_steps: int,
    x_range: tuple[float, float] = (0.10, 0.50),
    y_range: tuple[float, float] = (-0.20, 0.20),
    steps: int = 17,
) -> tuple[list[dict], dict]:
    """Where should the object stand?

    Scoring each pose against its own best hypothetical centre answers the
    wrong question: the object sits at one fixed place and every pose has to
    wrap *that*.  This sweeps candidate object positions on an x-y grid at the
    configured height and, for each, reports what the pose sweep can actually
    achieve there.  The support then goes where the evidence says, instead of
    where it looked plausible.
    """
    import itertools

    bend_lim = float(cfg.arm.limits.bend_per_node_rad)
    lin_lo, lin_hi = cfg.arm.limits.linear_m
    roll_lo, roll_hi = cfg.arm.limits.roll_rad
    bend_axis = torch.linspace(-bend_lim, bend_lim, grid)
    roll_axis = torch.linspace(float(roll_lo), float(roll_hi), roll_steps)
    lin_axis = torch.linspace(float(lin_lo), float(lin_hi), linear_steps)
    lin, roll, t1, t2 = torch.meshgrid(
        lin_axis, roll_axis, bend_axis, bend_axis, indexing="ij"
    )
    poses = torch.stack(
        (lin.reshape(-1), roll.reshape(-1), t1.reshape(-1), t2.reshape(-1)), dim=-1
    )
    n_envs = poses.shape[0]

    scene = WrapGraspScene(cfg, n_envs=n_envs).build()
    device = scene.device
    from .arm_kinematics import ArmWaypointMapper

    rate = cfg.macro_step.rate_limit
    mapper = ArmWaypointMapper(
        cfg.arm, n_envs=n_envs, device=device,
        rate_limit=(rate.linear_m, rate.roll_rad, rate.theta_rad, rate.theta_rad),
    )
    dofs = list(scene.arm_dofs.all_indices)
    _drive_to_pose(scene, mapper, poses.to(device), dofs)

    arm_ids = torch.tensor(sorted(scene.links.arm_local), device=device, dtype=torch.long)
    link_pos = scene.robot.get_links_pos()[:, arm_ids, :]

    # A pose that already collides with the floor, the support or the quadruped
    # is unusable no matter how much wrap it would score.  The environment
    # terminates on exactly this, so counting such poses as feasible would
    # overstate how many solutions the policy can actually reach.
    snap = ContactClassifier(scene).classify(n_envs)
    collides = (
        snap.floor_touch | snap.support_touch | snap.go2_touch | snap.self_touch
    )
    object_contacts = snap.object_link_hits.sum(dim=-1)
    collision_breakdown = {
        "floor": int(snap.floor_touch.sum()),
        "support": int(snap.support_touch.sum()),
        "quadruped": int(snap.go2_touch.sum()),
        "arm_self": int(snap.self_touch.sum()),
    }

    radius = float(cfg.object.radius_m)
    height = float(cfg.object.height_m)
    z = float(cfg.object_center()[2])
    link_r = float(cfg.reward.coverage.link_radius_m)
    tol = float(cfg.reward.coverage.interpenetration_tol_m)
    gate = float(cfg.success.coverage_target_rad)
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=device,
    )
    upright = torch.tensor([1.0, 0.0, 0.0, 0.0], device=device).expand(n_envs, 4)
    r_vec = torch.full((n_envs,), radius, device=device)
    h_vec = torch.full((n_envs,), height, device=device)

    xs = torch.linspace(*x_range, steps)
    ys = torch.linspace(*y_range, steps)
    rows: list[dict] = []
    for x, y in itertools.product(xs.tolist(), ys.tolist()):
        centre = torch.tensor([x, y, z], device=device).view(1, 3).expand(n_envs, 3)
        res = meter.measure(
            link_pos, centre, upright, radius_m=r_vec, height_m=h_vec,
            link_radius_m=link_r,
        )
        feasible = (res.min_surface_dist >= -tol) & ~collides
        phi = torch.where(feasible, res.phi_rad, torch.zeros_like(res.phi_rad))
        rows.append(
            {
                "x": round(x, 4),
                "y": round(y, 4),
                "z": round(z, 4),
                "phi_max_deg": round(math.degrees(float(phi.max())), 1),
                "n_at_gate": int((phi >= gate).sum()),
                "n_caged": int((feasible & res.caged).sum()),
            }
        )
    rows.sort(key=lambda r: (r["n_caged"], r["n_at_gate"], r["phi_max_deg"]), reverse=True)
    meta = {
        "n_poses": n_envs,
        "n_poses_colliding": int(collides.sum()),
        "collision_breakdown": collision_breakdown,
        "object_centre_z": z,
        "gate_deg": round(math.degrees(gate), 1),
        "grid_steps": steps,
        "x_range": list(x_range),
        "y_range": list(y_range),
    }
    return rows, meta

def render(results: list[PoseResult], meta: dict, cfg: WrapGraspConfig) -> str:
    target = float(cfg.success.coverage_target_rad)
    reach = [r for r in results if r.phi_rad >= target]
    lines = ["# Wrap workspace probe", ""]
    lines.append(
        "Peak wrap angle the 4-DoF arm reaches in the built Genesis scene, "
        "swept over the per-node bend limits with the best cylinder centre "
        "searched per pose."
    )
    lines.append("")
    lines.append("| item | value |")
    lines.append("|---|---|")
    lines.append(
        f"| grid (linear x roll x theta1 x theta2) | "
        f"{meta['linear_steps']} x {meta['roll_steps']} x {meta['grid']} x {meta['grid']} |"
    )
    lines.append(f"| poses evaluated | {meta['n_poses']} |")
    lines.append(
        f"| per-node bend limit | {math.degrees(meta['bend_per_node_rad']):.1f} deg |"
    )
    lines.append(f"| cylinder radius | {meta['object_radius_m'] * 1000:.0f} mm |")
    z = meta.get("object_centre_z")
    lines.append(
        f"| object centre height | {'free' if z is None else f'{z:.3f} m (fixed)'} |"
    )
    lines.append(f"| coverage bins | {meta['coverage_bins']} |")
    lines.append(f"| success gate | {math.degrees(target):.1f} deg |")
    lines.append(
        f"| poses at or above the gate | {len(reach)} / {meta['n_poses']} |"
    )
    caged = [r for r in results if r.caged]
    lines.append(f"| poses that actually cage the object | {len(caged)} / {meta['n_poses']} |")
    touching = meta.get("n_poses_touching_object")
    if touching is not None:
        lines.append(
            f"| poses in real contact with the object | {touching} / {meta['n_poses']} |"
        )
    lines.append("")
    lines.append("## Best poses")
    lines.append("")
    lines.append(
        "| linear (m) | roll (deg) | theta1 (deg/node) | theta2 (deg/node) | "
        "Phi (deg) | near links | caged | contacts | collides |"
    )
    lines.append("|---:|---:|---:|---:|---:|---:|:--|---:|:--|")
    for r in results[:15]:
        lines.append(
            f"| {r.linear:.3f} | {math.degrees(r.roll):.0f} | "
            f"{math.degrees(r.theta1):.1f} | {math.degrees(r.theta2):.1f} | "
            f"{math.degrees(r.phi_rad):.1f} | {r.n_near} | "
            f"{'yes' if r.caged else 'no'} | {r.n_contacts} | "
            f"{'yes' if r.collides else 'no'} |"
        )
    lines.append("")
    top = results[0]
    lines.append(
        f"Peak Phi = **{math.degrees(top.phi_rad):.1f} deg** at "
        f"linear = {top.linear:.3f} m, roll = {math.degrees(top.roll):.0f} deg, "
        f"theta1 = {math.degrees(top.theta1):.1f} deg/node, "
        f"theta2 = {math.degrees(top.theta2):.1f} deg/node, with the cylinder "
        f"centred at ({', '.join(f'{v:.3f}' for v in top.centre)}) m."
    )
    lines.append("")
    if not reach:
        lines.append(
            f"> No pose on this grid reaches the {math.degrees(target):.0f} deg "
            "gate.  The gate, the object radius, or the arm's reachable set has "
            "to change before a success bonus is attainable -- training against "
            "an unreachable gate would report zero success for a geometric "
            "reason, not a learning one."
        )
        lines.append("")
    return "\n".join(lines) + "\n"


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None)
    parser.add_argument("--overlay", action="append", default=[])
    parser.add_argument("--set", action="append", default=[], dest="overrides")
    parser.add_argument("--grid", type=int, default=9, help="bend-angle steps per segment")
    parser.add_argument("--roll-steps", type=int, default=9)
    parser.add_argument("--linear-steps", type=int, default=4)
    parser.add_argument(
        "--rank-by-contact",
        action="store_true",
        help="rank poses by real object contacts instead of geometric wrap",
    )
    parser.add_argument(
        "--placement-search",
        action="store_true",
        help="sweep candidate object positions and report what each affords",
    )
    parser.add_argument(
        "--at-config-centre",
        action="store_true",
        help="score only the configured object position, not the best one",
    )
    parser.add_argument(
        "--free-height",
        action="store_true",
        help="let the object float; reproduces the analytic sweep's assumption",
    )
    parser.add_argument("--out", default="sim/benchmarks/workspace.md")
    parser.add_argument("--json-out", default=None)
    args = parser.parse_args(argv)

    cfg = load_config(args.config, overlays=args.overlay, overrides=args.overrides)

    if args.placement_search:
        rows, pmeta = placement_search(
            cfg,
            grid=int(args.grid),
            roll_steps=int(args.roll_steps),
            linear_steps=int(args.linear_steps),
        )
        print(json.dumps({"meta": pmeta, "top": rows[:15]}, indent=2))
        out = Path(args.out)
        if not out.is_absolute():
            out = _REPO_ROOT / out
        out.parent.mkdir(parents=True, exist_ok=True)
        lines = ["# Object placement search", "", "| x (m) | y (m) | z (m) | max Phi (deg) | poses at gate | poses caged |", "|---:|---:|---:|---:|---:|---:|"]
        for r in rows[:25]:
            lines.append(
                f"| {r['x']:.3f} | {r['y']:.3f} | {r['z']:.3f} | "
                f"{r['phi_max_deg']:.1f} | {r['n_at_gate']} | {r['n_caged']} |"
            )
        lines.append("")
        lines.append(
            f"Pose sweep: {pmeta['n_poses']} poses, gate {pmeta['gate_deg']} deg, "
            f"object centre height {pmeta['object_centre_z']:.3f} m."
        )
        lines.append("")
        lines.append(
            f"{pmeta['n_poses_colliding']} of {pmeta['n_poses']} poses already "
            "collide with the floor, the support or the quadruped and are "
            "excluded: the environment terminates on those, so counting them "
            "would overstate the reachable solution set."
        )
        lines.append("")
        lines.append("| collision cause | poses |")
        lines.append("|---|---:|")
        for cause, count in pmeta["collision_breakdown"].items():
            lines.append(f"| `{cause}` | {count} |")
        out.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"[workspace] wrote {out}")
        return 0

    results, meta = probe(
        cfg,
        grid=int(args.grid),
        roll_steps=int(args.roll_steps),
        linear_steps=int(args.linear_steps),
        free_height=bool(args.free_height),
        at_config_centre=bool(args.at_config_centre),
        rank_by_contact=bool(args.rank_by_contact),
    )
    report = render(results, meta, cfg)

    out = Path(args.out)
    if not out.is_absolute():
        out = _REPO_ROOT / out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(report, encoding="utf-8")
    print(report)
    print(f"[workspace] wrote {out}")

    if args.json_out:
        payload = {
            "meta": meta,
            "results": [vars(r) for r in results],
        }
        jp = Path(args.json_out)
        if not jp.is_absolute():
            jp = _REPO_ROOT / jp
        jp.parent.mkdir(parents=True, exist_ok=True)
        jp.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")
        print(f"[workspace] wrote {jp}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
