"""Evaluate a trained wrap-grasp policy over a condition grid.

Reports success rate per condition and classifies every failure, because an
aggregate success rate cannot distinguish "the arm never arrived" from "it
wrapped and then dropped the object" -- and those call for different fixes.

Failure taxonomy:

``collision``  hit the floor, the support, the quadruped, or itself
``topple``     shoved or tipped the object before wrapping it
``retention``  wrapped and lifted, but did not hold on
``no_reach``   ran out of macro steps without ever touching the object
``no_wrap``    touched it, but never wrapped enough to attempt a lift

Run::

    python -m elesim_sim.rl.eval --checkpoint sim/rl_runs/.../model_600.pt
"""

from __future__ import annotations

import argparse
import itertools
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
from .headless_gl import select_offscreen_gl

_GL_PLATFORM = select_offscreen_gl()

import torch

from .configs.loader import load_config, to_dict, WrapGraspConfig
from .envs.wrap_env import WrapGraspEnv
from .scene import WrapGraspScene

_REPO_ROOT = Path(__file__).resolve().parents[4]

FAILURE_ORDER = ("collision", "topple", "retention", "no_wrap", "no_reach")


@dataclass
class ConditionResult:
    dx_m: float
    dy_m: float
    x_m: float
    y_m: float
    yaw_rad: float
    radius_m: float
    episodes: int = 0
    successes: int = 0
    failures: dict[str, int] = field(default_factory=lambda: {k: 0 for k in FAILURE_ORDER})
    phi_max_rad: float = 0.0
    phi_mean_rad: float = 0.0

    @property
    def success_rate(self) -> float:
        return self.successes / self.episodes if self.episodes else 0.0


class EpisodeTracker:
    """Per-env bookkeeping needed to classify an episode when it ends.

    The termination flags alone are not enough: an episode that simply timed
    out is a different failure depending on whether the arm ever reached the
    object, so contact and wrap history have to be carried through the episode.
    """

    def __init__(self, n_envs: int, device: torch.device) -> None:
        self.device = device
        self.touched = torch.zeros(n_envs, device=device, dtype=torch.bool)
        self.wrap_attempted = torch.zeros(n_envs, device=device, dtype=torch.bool)
        self.phi_max = torch.zeros(n_envs, device=device, dtype=torch.float32)
        self.phi_sum = torch.zeros(n_envs, device=device, dtype=torch.float32)
        self.steps = torch.zeros(n_envs, device=device, dtype=torch.float32)

    def update(self, env: WrapGraspEnv, phi: torch.Tensor) -> None:
        self.touched |= env.rewards.touched
        if env.lift is not None:
            self.wrap_attempted |= ~env.lift.follows_policy
        self.phi_max = torch.maximum(self.phi_max, phi)
        self.phi_sum += phi
        self.steps += 1.0

    def reset(self, env_ids: torch.Tensor) -> None:
        if env_ids.numel() == 0:
            return
        self.touched[env_ids] = False
        self.wrap_attempted[env_ids] = False
        self.phi_max[env_ids] = 0.0
        self.phi_sum[env_ids] = 0.0
        self.steps[env_ids] = 0.0


def classify(
    env: WrapGraspEnv,
    tracker: EpisodeTracker,
    reasons: dict[str, torch.Tensor],
    timeout: torch.Tensor,
    done_ids: torch.Tensor,
) -> tuple[int, dict[str, int]]:
    """Split finished episodes into one success bucket and five failure ones."""
    counts = {k: 0 for k in FAILURE_ORDER}
    success = int(reasons["success"][done_ids].sum())
    collision = reasons["collision"][done_ids] & ~reasons["success"][done_ids]
    topple = reasons["topple"][done_ids] & ~reasons["success"][done_ids] & ~collision
    counts["collision"] = int(collision.sum())
    counts["topple"] = int(topple.sum())
    other = timeout[done_ids] & ~reasons["success"][done_ids] & ~collision & ~topple
    if env.lift is not None:
        # An attempted lift that ended without success failed to retain.
        retention = other & tracker.wrap_attempted[done_ids]
        counts["retention"] = int(retention.sum())
        other = other & ~retention
    counts["no_wrap"] = int((other & tracker.touched[done_ids]).sum())
    counts["no_reach"] = int((other & ~tracker.touched[done_ids]).sum())
    return success, counts


def evaluate_condition(
    env: WrapGraspEnv,
    policy,
    *,
    dx_m: float,
    dy_m: float,
    yaw_rad: float,
    radius_m: float,
    episodes: int,
) -> ConditionResult:
    """Run `episodes` episodes with the object pinned to one condition."""
    centre = env.cfg.object_center()
    result = ConditionResult(
        dx_m=dx_m, dy_m=dy_m,
        x_m=float(centre[0]) + dx_m, y_m=float(centre[1]) + dy_m,
        yaw_rad=yaw_rad, radius_m=radius_m,
    )
    device = env.device
    tracker = EpisodeTracker(env.num_envs, device)

    # Pin the condition: every env gets the same object, so a batch of envs is
    # just a faster way to collect episodes of the same condition.
    env._eval_override = {
        "dx_m": dx_m, "dy_m": dy_m, "yaw_rad": yaw_rad, "radius_m": radius_m,
    }
    env.move_support_to(dx_m, dy_m)
    obs, _ = env.reset()
    tracker.reset(torch.arange(env.num_envs, device=device))

    phi_max = 0.0
    phi_total = 0.0
    phi_count = 0
    while result.episodes < episodes:
        with torch.inference_mode():
            actions = policy(obs)
        obs, rewards, dones, extras = env.step(actions)
        state_phi = env._read_state()["phi"]
        tracker.update(env, state_phi)
        phi_max = max(phi_max, float(state_phi.max()))
        phi_total += float(state_phi.mean())
        phi_count += 1

        done_ids = dones.nonzero(as_tuple=False).flatten()
        if done_ids.numel():
            reasons = extras.get("termination_reason") or env._last_reasons
            success, counts = classify(
                env, tracker, reasons, extras["time_outs"], done_ids
            )
            result.episodes += int(done_ids.numel())
            result.successes += success
            for key, value in counts.items():
                result.failures[key] += value
            tracker.reset(done_ids)

    result.phi_max_rad = phi_max
    result.phi_mean_rad = phi_total / max(phi_count, 1)
    env._eval_override = None
    env.move_support_to(0.0, 0.0)
    return result


def render_report(
    results: list[ConditionResult], cfg: WrapGraspConfig, checkpoint: str
) -> str:
    import math

    total_eps = sum(r.episodes for r in results)
    total_ok = sum(r.successes for r in results)
    lines = ["# Wrap-grasp policy evaluation", ""]
    lines.append(f"Checkpoint: `{checkpoint}`")
    lines.append("")
    lines.append("| item | value |")
    lines.append("|---|---|")
    lines.append(f"| success criterion | `{cfg.success.criterion}` |")
    lines.append(f"| curriculum stage | {cfg.curriculum.stage} |")
    lines.append(f"| GL platform | `{_GL_PLATFORM}` |")
    lines.append(f"| episodes | {total_eps} |")
    lines.append(
        f"| overall success | **{total_ok}/{total_eps}"
        f" = {100.0 * total_ok / max(total_eps, 1):.1f}%** |"
    )
    lines.append("")
    lines.append("## Per condition")
    lines.append("")
    lines.append(
        "| dx (m) | dy (m) | x (m) | yaw (deg) | radius (mm) | episodes | success | "
        + " | ".join(FAILURE_ORDER)
        + " | max Phi (deg) |"
    )
    lines.append("|---:|---:|---:|---:|---:|---:|---:|" + "---:|" * (len(FAILURE_ORDER) + 1))
    for r in results:
        lines.append(
            f"| {r.dx_m:+.3f} | {r.dy_m:+.3f} | {r.x_m:.3f} | "
            f"{math.degrees(r.yaw_rad):.0f} | {r.radius_m * 1000:.0f} | "
            f"{r.episodes} | {100.0 * r.success_rate:.0f}% | "
            + " | ".join(str(r.failures[k]) for k in FAILURE_ORDER)
            + f" | {math.degrees(r.phi_max_rad):.0f} |"
        )
    lines.append("")
    lines.append("## Failure meanings")
    lines.append("")
    lines.append("| bucket | meaning |")
    lines.append("|---|---|")
    lines.append("| `collision` | hit the floor, support, quadruped, or itself |")
    lines.append("| `topple` | shoved or tipped the object before wrapping |")
    lines.append("| `retention` | lifted, then failed to hold on |")
    lines.append("| `no_wrap` | touched the object but never wrapped enough to lift |")
    lines.append("| `no_reach` | never touched the object |")
    lines.append("")
    if total_ok == 0:
        lines.append(
            "> No successes. The failure columns say which stage the policy is "
            "stuck at; a run reporting all `no_reach` has not learned to "
            "approach, which is a different problem from one reporting all "
            "`retention`."
        )
        lines.append("")
    return "\n".join(lines) + "\n"


def _record_episode(
    env: WrapGraspEnv,
    policy,
    *,
    macro_steps: int,
    every: int,
    episodes: int,
) -> Any:
    """Record a rollout, sampling frames per *substep*.

    One frame per macro step is far too sparse to watch: a macro step covers
    `macro_step.substeps * dt` of simulated time -- 0.4 s at the defaults -- so
    a full 15-step episode collapses into 15 frames, and an episode that ends
    early into three.  Frames are taken inside the substep loop instead, via
    the environment's substep hook, which is also the only place the arm's
    motion between waypoints is visible at all.

    Recording spans `episodes` episodes so a policy that terminates early still
    produces something watchable; resets show as cuts.
    """
    camera = env.scene.cameras["eval"]
    camera.start_recording()
    env._eval_override = None

    stride = max(1, int(every))
    counter = {"substep": 0}

    def monitor(_env: WrapGraspEnv, _substep: int) -> None:
        if counter["substep"] % stride == 0:
            camera.render()
        counter["substep"] += 1

    previous_monitor = env.substep_monitor
    env.substep_monitor = monitor
    try:
        finished = 0
        obs, _ = env.reset()
        for _ in range(int(macro_steps)):
            with torch.inference_mode():
                actions = policy(obs)
            obs, _, dones, _ = env.step(actions)
            if bool(dones[0]):
                finished += 1
                if finished >= max(1, int(episodes)):
                    break
    finally:
        env.substep_monitor = previous_monitor
    return camera



def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--config", default=None)
    parser.add_argument("--overlay", action="append", default=[])
    parser.add_argument("--set", action="append", default=[], dest="overrides")
    parser.add_argument("--episodes", type=int, default=None)
    parser.add_argument("--out", default=None)
    parser.add_argument(
        "--render", type=int, default=0,
        help="record up to this many macro steps to an mp4",
    )
    parser.add_argument(
        "--render-every", type=int, default=4,
        help="capture a frame every Nth physics substep",
    )
    parser.add_argument(
        "--render-episodes", type=int, default=None,
        help="stop after this many episodes (default: eval.render_episodes)",
    )
    parser.add_argument(
        "--render-fps", type=int, default=None,
        help="output fps (default: real time for the substep stride)",
    )
    parser.add_argument(
        "--video-out", default=None, help="mp4 path (default: alongside the report)"
    )
    args = parser.parse_args(argv)

    cfg = load_config(args.config, overlays=args.overlay, overrides=args.overrides)

    camera_specs = []
    if args.render > 0:
        # Cameras must exist before scene.build(), so they are declared here and
        # the scene is constructed with them rather than attached afterwards.
        camera_specs = [
            {
                "name": "eval",
                "res": (960, 720),
                "pos": (1.25, -0.95, 1.05),
                "lookat": tuple(float(v) for v in cfg.object_center()),
                "fov": 40,
            }
        ]
    scene = WrapGraspScene(cfg, camera_specs=camera_specs).build()
    env = WrapGraspEnv(cfg, scene=scene)

    from rsl_rl.runners import OnPolicyRunner

    from .train import build_runner

    ckpt = Path(args.checkpoint)
    if not ckpt.is_absolute():
        ckpt = _REPO_ROOT / ckpt
    log_dir = ckpt.parent / "eval"
    log_dir.mkdir(parents=True, exist_ok=True)
    runner: OnPolicyRunner = build_runner(env, cfg, log_dir)
    runner.load(str(ckpt))
    policy = runner.get_inference_policy(device=env.device)

    ev = cfg.eval
    episodes = int(args.episodes or ev.episodes_per_condition)
    results: list[ConditionResult] = []
    grid = list(
        itertools.product(
            ev.pose_offset_grid.x_m,
            ev.pose_offset_grid.y_m,
            ev.pose_offset_grid.yaw_rad,
            ev.radius_grid_m,
        )
    )
    for index, (dx, dy, yaw, radius) in enumerate(grid, start=1):
        print(
            f"[eval] {index}/{len(grid)}  dx={dx:+.3f} dy={dy:+.3f} "
            f"yaw={yaw:+.2f} r={radius:.3f}",
            flush=True,
        )
        results.append(
            evaluate_condition(
                env, policy, dx_m=dx, dy_m=dy, yaw_rad=yaw,
                radius_m=radius, episodes=episodes,
            )
        )
        last = results[-1]
        print(
            f"[eval]     success {100.0 * last.success_rate:.0f}%  "
            f"failures {last.failures}",
            flush=True,
        )

    if args.render > 0:
        stride = max(1, int(args.render_every))
        # Default to real time: one frame per `stride` substeps of `dt` each.
        fps = args.render_fps or max(1, round(1.0 / (cfg.scene.dt * stride)))
        video = _record_episode(
            env,
            policy,
            macro_steps=int(args.render),
            every=stride,
            episodes=int(args.render_episodes or cfg.eval.render_episodes),
        )
        out_video = Path(args.video_out) if args.video_out else None
        if out_video is None:
            out_video = Path(cfg.eval.out_dir) / "eval_episode.mp4"
        if not out_video.is_absolute():
            out_video = _REPO_ROOT / out_video
        out_video.parent.mkdir(parents=True, exist_ok=True)
        video.stop_recording(save_to_filename=str(out_video), fps=int(fps))
        print(f"[eval] wrote {out_video} (every {stride} substeps, {fps} fps)")

    report = render_report(results, cfg, str(ckpt))
    out = Path(args.out) if args.out else Path(cfg.eval.out_dir) / "eval.md"
    if not out.is_absolute():
        out = _REPO_ROOT / out
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(report, encoding="utf-8")
    (out.with_suffix(".json")).write_text(
        json.dumps(
            {
                "checkpoint": str(ckpt),
                "config": to_dict(cfg),
                "conditions": [vars(r) for r in results],
            },
            indent=2,
            default=str,
        ),
        encoding="utf-8",
    )
    print(report)
    print(f"[eval] wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
