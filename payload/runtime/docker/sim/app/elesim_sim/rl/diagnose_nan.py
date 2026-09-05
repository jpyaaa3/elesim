"""Catch the state the rigid solver diverges on.

Training dies reproducibly with ``Invalid constraint forces causing 'nan'``,
and changing environment count, contact buffer, solver substeps, timestep and
the plate's inertia only moved *when* it happened.  That means the setting was
never the cause: the policy is finding a particular configuration the solver
cannot resolve.  This finds that configuration instead of guessing at it.

Two things are watched every substep, not every macro step -- the solver fails
inside a single substep, so anything coarser sees only the aftermath:

* a rolling buffer of recent states, dumped if Genesis raises;
* threshold trips on joint speed, contact force and penetration, which usually
  fire a few substeps *before* the values become non-finite and give a cleaner
  picture than the blown-up frame.

Run::

    python -m elesim_sim.rl.diagnose_nan --checkpoint .../model_10.pt
"""

from __future__ import annotations

import argparse
import json
import math
from collections import deque
from pathlib import Path
from typing import Any, Optional, Sequence

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import torch

from .configs.loader import load_config
from .envs.wrap_env import WrapGraspEnv

_REPO_ROOT = next(root for root in Path(__file__).resolve().parents if (root / "AGENTS.md").is_file())


class DivergenceWatcher:
    """Samples per-substep state and flags the first env to blow up."""

    def __init__(
        self,
        env: WrapGraspEnv,
        *,
        history: int = 12,
        vel_limit: float = 50.0,
        force_limit: float = 5000.0,
        penetration_limit: float = 0.05,
    ) -> None:
        self.env = env
        self.history: deque[dict[str, Any]] = deque(maxlen=history)
        self.vel_limit = vel_limit
        self.force_limit = force_limit
        self.penetration_limit = penetration_limit
        self.trip: Optional[dict[str, Any]] = None
        self.macro_step = 0

    def __call__(self, env: WrapGraspEnv, substep: int) -> None:
        scene = env.scene
        dof_pos = scene.robot.get_dofs_position(dofs_idx_local=env._arm_dofs)
        dof_vel = scene.robot.get_dofs_velocity(dofs_idx_local=env._arm_dofs)
        dof_force = scene.robot.get_dofs_force(dofs_idx_local=env._arm_dofs)
        raw = scene.robot.get_contacts(exclude_self_contact=False)
        valid = raw["valid_mask"]
        if valid.shape[-1]:
            force_mag = raw["force_a"].norm(dim=-1) * valid
            pen = raw["penetration"].abs() * valid
        else:
            force_mag = torch.zeros_like(dof_vel[:, :1])
            pen = torch.zeros_like(dof_vel[:, :1])

        finite = (
            torch.isfinite(dof_pos).all(dim=-1)
            & torch.isfinite(dof_vel).all(dim=-1)
            & torch.isfinite(dof_force).all(dim=-1)
        )
        vel_max = dof_vel.abs().amax(dim=-1)
        force_max = force_mag.amax(dim=-1)
        pen_max = pen.amax(dim=-1)

        snap = {
            "macro_step": self.macro_step,
            "substep": substep,
            "worst_env": int(vel_max.argmax()),
            "vel_max": float(vel_max.max()),
            "contact_force_max": float(force_max.max()),
            "penetration_max_mm": float(pen_max.max()) * 1000.0,
            "dof_force_max": float(dof_force.abs().max()),
            "n_nonfinite_envs": int((~finite).sum()),
        }
        self.history.append(snap)

        if self.trip is not None:
            return
        bad = (
            (~finite).any()
            or bool((vel_max > self.vel_limit).any())
            or bool((force_max > self.force_limit).any())
            or bool((pen_max > self.penetration_limit).any())
        )
        if bad:
            e = int(torch.argmax(torch.where(finite, vel_max, torch.full_like(vel_max, 1e9))))
            self.trip = {
                "reason": (
                    "non-finite state" if bool((~finite).any())
                    else "threshold exceeded"
                ),
                **snap,
                "env": e,
                "waypoint": [round(float(v), 5) for v in env.mapper.waypoint[e]],
                "dof_position": [round(float(v), 5) for v in dof_pos[e]],
                "dof_velocity": [round(float(v), 4) for v in dof_vel[e]],
                "dof_force": [round(float(v), 3) for v in dof_force[e]],
                "object_pos": [round(float(v), 5) for v in scene.object.get_pos()[e]],
                "contacts": self._contact_rows(raw, e),
            }

    def _contact_rows(self, raw: dict[str, torch.Tensor], env_index: int) -> list[dict]:
        valid = raw["valid_mask"]
        if not valid.shape[-1]:
            return []
        names = self.env.scene.links.name_by_index
        rows = []
        for k in range(valid.shape[-1]):
            if not bool(valid[env_index, k]):
                continue
            a = int(raw["link_a"][env_index, k])
            b = int(raw["link_b"][env_index, k])
            rows.append(
                {
                    "a": names.get(a, str(a)),
                    "b": names.get(b, str(b)),
                    "penetration_mm": round(float(raw["penetration"][env_index, k]) * 1000, 3),
                    "force": round(float(raw["force_a"][env_index, k].norm()), 2),
                }
            )
        rows.sort(key=lambda r: abs(r["penetration_mm"]), reverse=True)
        return rows[:12]


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--config", default=None)
    parser.add_argument("--set", action="append", default=[], dest="overrides")
    parser.add_argument("--max-macro-steps", type=int, default=4000)
    parser.add_argument("--out", default="workbench/evidence/generated/sim/divergence.json")
    args = parser.parse_args(argv)

    cfg = load_config(args.config, overrides=args.overrides)
    env = WrapGraspEnv(cfg)
    watcher = DivergenceWatcher(env)
    env.substep_monitor = watcher

    policy = None
    if args.checkpoint:
        from .train import build_runner

        ckpt = Path(args.checkpoint)
        if not ckpt.is_absolute():
            ckpt = _REPO_ROOT / ckpt
        runner = build_runner(env, cfg, ckpt.parent / "diagnose")
        runner.load(str(ckpt))
        policy = runner.get_inference_policy(device=env.device)
        print(f"[diagnose] policy from {ckpt.name}")
    else:
        print("[diagnose] no checkpoint: driving with random actions")

    obs, _ = env.reset()
    payload: dict[str, Any] = {"config_overrides": list(args.overrides)}
    try:
        for step in range(int(args.max_macro_steps)):
            watcher.macro_step = step
            if policy is not None:
                with torch.inference_mode():
                    actions = policy(obs)
            else:
                actions = torch.rand(env.num_envs, 4, device=env.device) * 2 - 1
            obs, rewards, dones, extras = env.step(actions)
            if watcher.trip is not None:
                payload["outcome"] = "threshold trip"
                break
        else:
            payload["outcome"] = "completed without diverging"
    except Exception as exc:  # noqa: BLE001 - the exception is the result
        payload["outcome"] = "exception"
        payload["exception"] = f"{type(exc).__name__}: {exc}"

    payload["trip"] = watcher.trip
    payload["history"] = list(watcher.history)
    out = Path(args.out)
    if not out.is_absolute():
        out = _REPO_ROOT / out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2, default=str), encoding="utf-8")

    print(f"[diagnose] outcome: {payload['outcome']}")
    if payload.get("exception"):
        print(f"[diagnose] {payload['exception']}")
    trip = watcher.trip
    if trip:
        print(f"[diagnose] first trip: {trip['reason']} at macro step "
              f"{trip['macro_step']}, substep {trip['substep']}, env {trip['env']}")
        print(f"           joint speed max {trip['vel_max']:.1f} rad/s, "
              f"contact force max {trip['contact_force_max']:.0f} N, "
              f"penetration max {trip['penetration_max_mm']:.1f} mm")
        wp = trip["waypoint"]
        print(f"           waypoint: linear={wp[0]:+.3f} m roll={math.degrees(wp[1]):+.1f} "
              f"t1={math.degrees(wp[2]):+.1f} t2={math.degrees(wp[3]):+.1f} deg/node")
        print("           worst contacts:")
        for row in trip["contacts"][:6]:
            print(f"             {row['a']:>18s} <-> {row['b']:<18s} "
                  f"pen={row['penetration_mm']:+8.2f} mm  F={row['force']:8.1f}")
    print(f"[diagnose] wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
