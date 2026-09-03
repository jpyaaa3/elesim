"""Render the wrap scene to still images.

Numbers alone were a poor way to debug this geometry: the object placement and
the coverage metric were both wrong in ways a single picture would have shown
immediately.  This renders the built scene from several viewpoints, optionally
after driving the arm to a given 4-DoF pose, so a claimed wrap can be looked at
rather than inferred from a wrap angle.

Run::

    python -m elesim_sim.rl.render_scene --out misc/research/sim/benchmarks/render
    python -m elesim_sim.rl.render_scene --pose 0.0 -1.5708 0.4712 0.4712
    python -m elesim_sim.rl.render_scene --top-contact   # best contacting pose
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Optional, Sequence

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
from .headless_gl import select_offscreen_gl

_GL_PLATFORM = select_offscreen_gl()  # must precede the genesis import

import numpy as np
import torch

from .arm_kinematics import ArmWaypointMapper
from .configs.loader import load_config, WrapGraspConfig
from .envs.contacts import ContactClassifier
from .envs.coverage import CoverageMeter
from .scene import WrapGraspScene

_REPO_ROOT = next(root for root in Path(__file__).resolve().parents if (root / "AGENTS.md").is_file())

#: Viewpoints chosen to show the three things that went wrong before: whether
#: the object is reachable at all, whether the arm closes around it, and
#: whether it is hitting the support or the quadruped.
_VIEWS: tuple[tuple[str, tuple[float, float, float], tuple[float, float, float]], ...] = (
    ("iso", (1.1, -0.9, 1.0), (0.22, -0.10, 0.50)),
    ("front", (1.3, -0.12, 0.58), (0.10, -0.12, 0.52)),
    ("side", (0.22, -1.2, 0.62), (0.22, -0.12, 0.55)),
    ("top", (0.24, -0.12, 1.5), (0.24, -0.12, 0.50)),
    ("wide", (1.8, -1.6, 1.3), (0.15, -0.05, 0.35)),
)


def _save_png(rgb: np.ndarray, path: Path) -> None:
    from PIL import Image

    array = np.asarray(rgb)
    if array.dtype != np.uint8:
        array = np.clip(array, 0.0, 1.0)
        array = (array * 255).astype(np.uint8)
    Image.fromarray(array[..., :3]).save(path)


def _resolve_pose(cfg: WrapGraspConfig, args) -> tuple[torch.Tensor, str]:
    if args.top_contact:
        path = Path(args.contact_json)
        if not path.is_absolute():
            path = _REPO_ROOT / path
        with open(path, "r", encoding="utf-8") as handle:
            payload = json.load(handle)
        best = max(payload["results"], key=lambda r: r.get("n_contacts", 0))
        pose = torch.tensor(
            [best["linear"], best["roll"], best["theta1"], best["theta2"]],
            dtype=torch.float32,
        )
        label = (
            f"top-contact ({best.get('n_contacts', 0)} contacts, "
            f"roll={math.degrees(best['roll']):.0f}deg)"
        )
        return pose, label
    if args.pose:
        return torch.tensor([float(v) for v in args.pose], dtype=torch.float32), "given"
    return torch.zeros(4, dtype=torch.float32), "home"


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=None)
    parser.add_argument("--overlay", action="append", default=[])
    parser.add_argument("--set", action="append", default=[], dest="overrides")
    parser.add_argument(
        "--pose", nargs=4, type=float, default=None,
        metavar=("LINEAR", "ROLL", "THETA1", "THETA2"),
        help="4-DoF waypoint to drive to before rendering",
    )
    parser.add_argument(
        "--top-contact", action="store_true",
        help="use the highest-contact pose from a workspace probe JSON",
    )
    parser.add_argument("--contact-json", default="misc/research/sim/benchmarks/workspace.json")
    parser.add_argument("--out", default="misc/research/sim/benchmarks/render")
    parser.add_argument("--res", type=int, nargs=2, default=(960, 720))
    parser.add_argument("--ramp", type=int, default=120)
    parser.add_argument("--tag", default=None)
    parser.add_argument(
        "--place-object-after", action="store_true",
        help="pose the arm first, then set the object at its configured centre",
    )
    args = parser.parse_args(argv)

    cfg = load_config(args.config, overlays=args.overlay, overrides=args.overrides)
    pose, label = _resolve_pose(cfg, args)

    specs = [
        {
            "name": name,
            "res": tuple(int(v) for v in args.res),
            "pos": pos,
            "lookat": look,
            "fov": 40,
        }
        for name, pos, look in _VIEWS
    ]
    scene = WrapGraspScene(cfg, n_envs=1, camera_specs=specs).build()
    cameras = list(scene.cameras.items())

    rate = cfg.macro_step.rate_limit
    mapper = ArmWaypointMapper(
        cfg.arm, n_envs=1, device=scene.device,
        rate_limit=(rate.linear_m, rate.roll_rad, rate.theta_rad, rate.theta_rad),
    )
    dofs = list(scene.arm_dofs.all_indices)
    target = pose.to(scene.device).view(1, 4)
    home = torch.zeros_like(target)
    for i in range(int(args.ramp)):
        alpha = float(i + 1) / int(args.ramp)
        scene.robot.control_dofs_position(
            mapper.joint_targets(home + (target - home) * alpha), dofs_idx_local=dofs
        )
        scene.step()
    final = mapper.joint_targets(target)
    for _ in range(60):
        scene.robot.control_dofs_position(final, dofs_idx_local=dofs)
        scene.step()

    if args.place_object_after:
        # Answers the static question -- "with the object where it belongs, is
        # this pose a wrap?" -- separately from the dynamic one, which is
        # whether the arm can get there without sweeping the object out of the
        # way.  The second is the policy's problem; conflating them makes a
        # correct pose look wrong.
        centre = torch.tensor(
            [[float(v) for v in cfg.object_center()]], device=scene.device
        )
        scene.object.set_pos(centre)
        scene.object.set_quat(
            torch.tensor([[1.0, 0.0, 0.0, 0.0]], device=scene.device)
        )
        for setter in ("set_vel", "set_ang"):
            fn = getattr(scene.object, setter, None)
            if callable(fn):
                try:
                    fn(torch.zeros((1, 3), device=scene.device))
                except Exception:
                    pass
        for _ in range(30):
            scene.robot.control_dofs_position(final, dofs_idx_local=dofs)
            scene.step()

    # Report what the picture is showing, so an image and its numbers cannot
    # drift apart.
    snap = ContactClassifier(scene).classify(1)
    arm_ids = torch.tensor(sorted(scene.links.arm_local), device=scene.device, dtype=torch.long)
    meter = CoverageMeter(
        n_bins=cfg.reward.coverage.n_bins,
        radial_band_m=cfg.reward.coverage.radial_band_m,
        device=scene.device,
    )
    cov = meter.measure(
        scene.robot.get_links_pos()[:, arm_ids, :],
        scene.object.get_pos(),
        scene.object.get_quat(),
        radius_m=torch.tensor([cfg.object.radius_m], device=scene.device),
        height_m=torch.tensor([cfg.object.height_m], device=scene.device),
        link_radius_m=cfg.reward.coverage.link_radius_m,
    )
    facts = {
        "pose": label,
        "waypoint": [round(float(v), 4) for v in pose],
        "object_centre": [round(float(v), 4) for v in scene.object.get_pos()[0]],
        "phi_deg": round(math.degrees(float(cov.phi_rad[0])), 1),
        "near_links": int(cov.n_near_links[0]),
        "caged": bool(cov.caged[0]),
        "object_contacts": int(snap.object_link_hits[0].sum()),
        "floor_or_support_touch": bool(snap.floor_touch[0]),
        "quadruped_touch": bool(snap.go2_touch[0]),
        "arm_self_touch": bool(snap.self_touch[0]),
    }
    facts["gl_platform"] = _GL_PLATFORM
    print(json.dumps(facts, indent=2))

    out_dir = Path(args.out)
    if not out_dir.is_absolute():
        out_dir = _REPO_ROOT / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = args.tag or ("top_contact" if args.top_contact else ("pose" if args.pose else "home"))
    for name, cam in cameras:
        rgb = cam.render()
        if isinstance(rgb, tuple):
            rgb = rgb[0]
        path = out_dir / f"{tag}_{name}.png"
        _save_png(rgb, path)
        print(f"[render] {path}")
    (out_dir / f"{tag}_facts.json").write_text(
        json.dumps(facts, indent=2), encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
