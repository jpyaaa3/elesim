"""Export successful wrap-grasp episodes as waypoint sequences.

The point of training in simulation is to get a trajectory the real arm can
replay **open loop**, because the hardware has no contact sensing.  So what is
exported is exactly what the real arm can be commanded with: the per-macro-step
4-DoF waypoint, in physical units, plus enough context to tell whether a replay
is being attempted under the conditions the trajectory was found in.

Nothing simulator-only goes into the trajectory columns themselves -- no contact
forces, no true object pose.  Those appear only in a separate diagnostics block,
clearly marked, so a replay script cannot accidentally consume them.

Run::

    python -m elesim_sim.rl.export_traj --checkpoint .../model_600.pt --count 20
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import torch

from .configs.loader import load_config, to_dict
from .envs.wrap_env import WrapGraspEnv

_REPO_ROOT = Path(__file__).resolve().parents[4]

#: Trajectory columns.  These are commands, in SI units, and nothing else.
COLUMNS = ("step", "linear_m", "roll_rad", "theta1_rad", "theta2_rad")


@dataclass
class Episode:
    """One successful attempt, as commands plus separate diagnostics."""

    env_index: int
    waypoints: list[list[float]] = field(default_factory=list)
    object_radius_m: float = 0.0
    object_height_m: float = 0.0
    object_start_xyz: tuple[float, float, float] = (0.0, 0.0, 0.0)
    object_start_yaw_rad: float = 0.0
    #: Simulator-only, for inspection.  Never part of the replay command set.
    diagnostics: dict = field(default_factory=dict)

    def as_rows(self) -> list[dict]:
        return [
            dict(zip(COLUMNS, [index, *wp])) for index, wp in enumerate(self.waypoints)
        ]


class Recorder:
    """Buffers each env's waypoints and keeps the ones that succeed."""

    def __init__(self, env: WrapGraspEnv) -> None:
        self.env = env
        self.n = env.num_envs
        self.buffers: list[list[list[float]]] = [[] for _ in range(self.n)]
        self.start_state: list[dict] = [{} for _ in range(self.n)]
        self.captured: list[Episode] = []

    def note_reset(self, env_ids: Sequence[int]) -> None:
        obj_pos = self.env.scene.object.get_pos()
        for i in env_ids:
            self.buffers[i] = []
            self.start_state[i] = {
                "radius_m": float(self.env._object_radius[i]),
                "height_m": float(self.env._object_height[i]),
                "xyz": tuple(float(v) for v in obj_pos[i]),
            }

    def record(self, waypoints: torch.Tensor) -> None:
        wp = waypoints.detach().to("cpu")
        for i in range(self.n):
            self.buffers[i].append([round(float(v), 6) for v in wp[i]])

    def harvest(self, done_ids: torch.Tensor, success: torch.Tensor, phi: torch.Tensor) -> int:
        taken = 0
        for i in done_ids.tolist():
            if not bool(success[i]):
                continue
            start = self.start_state[i] or {}
            self.captured.append(
                Episode(
                    env_index=int(i),
                    waypoints=list(self.buffers[i]),
                    object_radius_m=float(start.get("radius_m", 0.0)),
                    object_height_m=float(start.get("height_m", 0.0)),
                    object_start_xyz=tuple(start.get("xyz", (0.0, 0.0, 0.0))),
                    diagnostics={
                        "macro_steps": len(self.buffers[i]),
                        "final_wrap_deg": round(math.degrees(float(phi[i])), 2),
                        "note": "simulator-only; not part of the replay command set",
                    },
                )
            )
            taken += 1
        return taken


def write_outputs(episodes: list[Episode], out_dir: Path, cfg) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    for index, ep in enumerate(episodes):
        csv_path = out_dir / f"traj_{index:03d}.csv"
        with open(csv_path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=COLUMNS)
            writer.writeheader()
            writer.writerows(ep.as_rows())

    payload = {
        "schema": {
            "columns": list(COLUMNS),
            "units": "linear_m in metres; roll/theta1/theta2 in radians",
            "semantics": (
                "One row per macro step: the commanded 4-DoF waypoint after that "
                "step's increment. Replay open loop by driving the arm to each "
                "waypoint in order and letting it settle before the next."
            ),
            "theta_convention": (
                "theta1/theta2 are PER-NODE angles; every node in a segment takes "
                "the same value, which is what makes each segment a constant-"
                "curvature arc. Segment total is n_seg times the value."
            ),
            "n_seg": int(cfg.arm.n_seg),
            "joint_order": [cfg.arm.linear_joint, cfg.arm.roll_joint]
            + list(cfg.arm.bend_joints),
        },
        "provenance": {
            "beta": (
                "placeholder (config)" if not cfg.beta.measured else "measured"
            ),
            "beta_note": (
                "Trajectories were found under a residual model whose parameters "
                "are placeholders, not hardware identification. Replay accuracy "
                "on the real arm is unverified."
            ),
            "success_criterion": cfg.success.criterion,
            "curriculum_stage": int(cfg.curriculum.stage),
        },
        "episodes": [
            {
                "file": f"traj_{index:03d}.csv",
                "macro_steps": len(ep.waypoints),
                "object": {
                    "radius_m": ep.object_radius_m,
                    "height_m": ep.object_height_m,
                    "start_xyz": list(ep.object_start_xyz),
                },
                "waypoints": ep.waypoints,
                "diagnostics": ep.diagnostics,
            }
            for index, ep in enumerate(episodes)
        ],
        "config": to_dict(cfg),
    }
    (out_dir / "trajectories.json").write_text(
        json.dumps(payload, indent=2, default=str), encoding="utf-8"
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--config", default=None)
    parser.add_argument("--overlay", action="append", default=[])
    parser.add_argument("--set", action="append", default=[], dest="overrides")
    parser.add_argument("--count", type=int, default=10, help="successes to collect")
    parser.add_argument("--max-steps", type=int, default=4000, help="give-up budget")
    parser.add_argument("--out", default="sim/rl_runs/export")
    args = parser.parse_args(argv)

    cfg = load_config(args.config, overlays=args.overlay, overrides=args.overrides)
    env = WrapGraspEnv(cfg)

    from .train import build_runner

    ckpt = Path(args.checkpoint)
    if not ckpt.is_absolute():
        ckpt = _REPO_ROOT / ckpt
    runner = build_runner(env, cfg, ckpt.parent / "export")
    runner.load(str(ckpt))
    policy = runner.get_inference_policy(device=env.device)

    recorder = Recorder(env)
    obs, _ = env.reset()
    recorder.note_reset(range(env.num_envs))

    steps = 0
    while len(recorder.captured) < args.count and steps < int(args.max_steps):
        with torch.inference_mode():
            actions = policy(obs)
        obs, rewards, dones, extras = env.step(actions)
        recorder.record(env.mapper.waypoint)
        steps += 1

        done_ids = dones.nonzero(as_tuple=False).flatten()
        if done_ids.numel():
            success = extras["termination_reason"]["success"]
            phi = env._read_state()["phi"]
            got = recorder.harvest(done_ids, success, phi)
            if got:
                print(
                    f"[export] {len(recorder.captured)}/{args.count} captured "
                    f"(step {steps})",
                    flush=True,
                )
            recorder.note_reset(done_ids.tolist())

    episodes = recorder.captured[: args.count]
    out_dir = Path(args.out)
    if not out_dir.is_absolute():
        out_dir = _REPO_ROOT / out_dir
    write_outputs(episodes, out_dir, cfg)

    if not episodes:
        # Say so rather than writing an empty directory that reads as success.
        print(
            f"[export] no successful episodes in {steps} macro steps. "
            "Nothing was exported."
        )
        return 1
    print(f"[export] wrote {len(episodes)} trajectories to {out_dir}")
    for index, ep in enumerate(episodes):
        print(
            f"          traj_{index:03d}.csv  {len(ep.waypoints)} steps  "
            f"final wrap {ep.diagnostics['final_wrap_deg']:.0f} deg"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
