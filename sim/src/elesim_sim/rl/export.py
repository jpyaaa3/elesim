"""Export a trained policy for the real arm, with the interface it assumes.

Two files come out: a TorchScript module that maps the 16 observations to the 5
actions, and a manifest describing everything the robot side has to reproduce.

The manifest is not documentation, it is the contract.  A policy is a function
of a vector whose meaning lives entirely in the code that built it: put the
object's radius where the roll angle belongs and it still returns five numbers.
Nothing about the file itself would say so, which is why the layout, the units,
the rate limits, the clamps and the lift script travel with the weights.

The observation normalisation goes into the TorchScript module -- rsl_rl's
`as_jit` copies the normaliser in -- so the robot feeds it raw values.  Sending
raw values to a policy exported without it silently rescales every input.

Run::

    python -m elesim_sim.rl.export --checkpoint .../model_best.pt --out-dir deploy/
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
from pathlib import Path
from typing import Any, Optional, Sequence

_REPO_ROOT = Path(__file__).resolve().parents[4]


def _git_commit() -> Optional[str]:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=_REPO_ROOT,
            capture_output=True, text=True, timeout=5,
        )
        return out.stdout.strip() or None
    except Exception:
        return None


def observation_layout(cfg: Any) -> list[dict[str, Any]]:
    """The 16 actor observations, in the order the network expects them.

    Read off `WrapGraspEnv._actor_observation`, which appends the enabled groups
    in this order.  The actor never sees contact forces or the object's true
    pose: those are the critic's, which is what makes this deployable at all.
    """
    a = cfg.observation.actor
    out: list[dict[str, Any]] = []
    if a.include_joint_estimate:
        out += [
            {"name": "joint/linear", "unit": "m",
             "source": "linear stage position, beta-compensated"},
            {"name": "joint/roll", "unit": "rad",
             "source": "roll joint angle, beta-compensated"},
            {"name": "joint/theta1", "unit": "rad",
             "source": "mean of segment 1's node angles, beta-compensated"},
            {"name": "joint/theta2", "unit": "rad",
             "source": "mean of segment 2's node angles, beta-compensated"},
        ]
    if a.include_object_geometry:
        out += [
            {"name": "object/radius", "unit": "m", "source": "perception"},
            {"name": "object/height", "unit": "m", "source": "perception"},
            {"name": "object/pos_x", "unit": "m", "source": "perception, robot frame"},
            {"name": "object/pos_y", "unit": "m", "source": "perception, robot frame"},
            {"name": "object/pos_z", "unit": "m", "source": "perception, robot frame"},
            {"name": "object/lean_x", "unit": "-",
             "source": "object axis unit vector, x component"},
            {"name": "object/lean_y", "unit": "-",
             "source": "object axis unit vector, y component"},
        ]
    if a.include_load_proxy:
        out += [
            {"name": "load/linear", "unit": "N", "source": "linear stage load"},
            {"name": "load/roll", "unit": "Nm", "source": "roll joint load"},
            {"name": "load/bend", "unit": "Nm",
             "source": "mean load over the bend joints"},
            {"name": "load/bend_repeat", "unit": "Nm",
             "source": "the same value again -- the sim reports one bend load "
                       "into two channels, so the robot must too"},
        ]
    if a.include_step_index:
        out += [{"name": "episode/progress", "unit": "-",
                 "source": "macro steps taken / max_steps"}]
    return out


def build_manifest(cfg: Any, checkpoint: Path, obs_dim: int, action_dim: int) -> dict:
    arm, ms, lift = cfg.arm, cfg.macro_step, cfg.success.lift
    lo_lin, hi_lin = arm.limits.linear_m
    lo_roll, hi_roll = arm.limits.roll_rad
    rate = ms.rate_limit
    roll_secs = (
        abs(hi_roll - lift.roll_target_rad) / lift.roll_rate_rad_per_substep * cfg.scene.dt
    )
    return {
        "checkpoint": str(checkpoint),
        "git_commit": _git_commit(),
        "observation": {
            "dim": obs_dim,
            "note": "raw values -- normalisation is inside policy.pt",
            "channels": observation_layout(cfg),
        },
        "action": {
            "dim": action_dim,
            "note": "first four are waypoint increments in [-1, 1] scaled by "
                    "rate_limit; the fifth is the lift request, positive means "
                    "lift now",
            "channels": (
                [{"name": "delta/linear", "scale_m": rate.linear_m},
                 {"name": "delta/roll", "scale_rad": rate.roll_rad},
                 {"name": "delta/theta1", "scale_rad": rate.theta_rad},
                 {"name": "delta/theta2", "scale_rad": rate.theta_rad}]
                + ([{"name": "lift_request", "threshold": 0.0}]
                   if action_dim > 4 else [])
            ),
        },
        "waypoint": {
            "home": [float(v) for v in cfg.arm.home_waypoint or ()] or None,
            "limits": {
                "linear_m": [lo_lin, hi_lin],
                "roll_rad": [lo_roll, hi_roll],
                "theta_rad": [-arm.limits.bend_per_node_rad,
                              arm.limits.bend_per_node_rad],
            },
            "coupled_curl_cap": {
                "rule": "abs(theta1_weight * theta1 + theta2) <= cap",
                "theta1_weight": arm.limits.theta1_curl_weight,
                "cap_rad": arm.limits.curl_limit_per_node_rad,
                "why": "the backbone reaches its own housing past this; swept "
                       "on a 9x9 grid, every folding cell is separated exactly "
                       "by this bound",
            },
            "sign_conventions": {
                "linear_axis_sign": arm.linear_axis_sign,
                "roll_axis_sign": arm.roll_axis_sign,
                "bend_axis_sign": arm.bend_axis_sign,
                "warning": "runtime.JointLayout uses bend_axis_sign -1 where "
                           "this uses +1; the waypoints here are already in the "
                           "convention control_u_to_sim_q produces. Getting it "
                           "wrong folds the arm through the quadruped.",
            },
        },
        "timing": {
            "macro_step_s": ms.substeps * cfg.scene.dt,
            "substeps": ms.substeps,
            "substep_s": cfg.scene.dt,
            "move_fraction": ms.move_fraction,
            "note": "the waypoint is interpolated over the first "
                    "move_fraction of the step and held for the rest",
            "max_steps": ms.max_steps,
        },
        "lift_script": {
            "note": "the policy chooses when; the roll-back itself is scripted "
                    "and has to be reimplemented on the robot",
            "roll_target_rad": lift.roll_target_rad,
            "roll_rate_rad_per_substep": lift.roll_rate_rad_per_substep,
            "roll_seconds_for_90deg": roll_secs,
            "settle_substeps": lift.settle_substeps,
            "hold_substeps": lift.hold_substeps,
            "speed_matters": "measured on 32 envs: 0.31 s holds 0%, 0.52 s 0%, "
                             "0.79 s 3%, 1.05 s 72%, 1.57 s 75%",
            "wrap_angle_floor_rad": lift.trigger_rad,
            "wrap_angle_floor_on_robot": "not available -- the wrap angle is "
                                         "computed from simulator contacts. On "
                                         "the robot the policy's request is the "
                                         "only gate.",
        },
        "trained_under": {
            "object_radius_m": list(cfg.domain_randomisation.object_radius_m),
            "object_mass_kg": list(cfg.domain_randomisation.object_mass_kg),
            "object_pos_jitter_m": list(cfg.domain_randomisation.object_pos_jitter_m),
            "surface_friction": cfg.scene.friction,
            "joint_residual_deg": {
                "beta0": cfg.beta.beta0_deg,
                "jitter": cfg.beta.beta0_jitter_deg,
                "estimator_gain": cfg.beta.estimator_gain,
                "tracking_tolerance": "measured: no loss up to 2 deg of joint "
                                      "error, 3 deg still clean at 100 mm, "
                                      "breaks at 4 deg",
            },
            "observation_noise": {
                "joint_rad": cfg.observation.actor.noise.joint_rad,
                "object_pos_m": cfg.observation.actor.noise.object_pos_m,
            },
        },
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--out-dir", default="deploy")
    parser.add_argument("--config", default=None)
    parser.add_argument("--overlay", action="append", default=[])
    parser.add_argument("--set", action="append", default=[], dest="overrides")
    args = parser.parse_args(argv)

    import torch

    from .configs.loader import load_config

    # Export on the CPU, backend included.  rsl_rl's normaliser keeps its
    # statistics in plain attributes rather than buffers, so `.to("cpu")` after
    # the fact leaves them on the training device and the first forward pass
    # dies with "found at least two devices, mps:0 and cpu"; and the env's
    # device comes from the Genesis backend, not from `torch_device`, so
    # lowering only the latter gets "Placeholder storage has not been allocated
    # on MPS device".  The exported file has to be device-free anyway.
    overrides = list(args.overrides) + [
        "runtime.torch_device=cpu", "runtime.backend=cpu"
    ]
    cfg = load_config(args.config, overlays=args.overlay,
                      overrides=overrides).resolved_for_curriculum()
    ckpt = Path(args.checkpoint).expanduser()
    if not ckpt.is_absolute():
        here = Path.cwd() / ckpt
        ckpt = here if here.is_file() else _REPO_ROOT / ckpt
    if not ckpt.is_file():
        raise SystemExit(f"체크포인트가 없습니다: {ckpt}")

    out = Path(args.out_dir).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    # The scene is built because the runner needs an env to size the networks,
    # which is also the only honest way to learn the observation width: it is a
    # property of the config, and reading it off anywhere else could disagree.
    from .envs.wrap_env import WrapGraspEnv
    from .train import build_runner

    env = WrapGraspEnv(cfg, n_envs=1)
    runner = build_runner(env, cfg, out)
    runner.load(str(ckpt), map_location="cpu")
    policy = runner.alg.get_policy()
    jit = policy.as_jit().to("cpu").eval()

    obs_dim = int(env.obs_spec.policy)
    action_dim = int(env.num_actions)
    probe = torch.zeros(1, obs_dim)
    with torch.no_grad():
        acted = jit(probe)
    if tuple(acted.shape) != (1, action_dim):
        raise SystemExit(
            f"내보낸 정책의 출력이 {tuple(acted.shape)} 인데 "
            f"행동 차원은 {action_dim} 입니다"
        )
    torch.jit.save(torch.jit.script(jit), out / "policy.pt")

    # ...and the same weights as plain arrays, because the control computer is a
    # Jetson AGX Orin.  Matching a torch build to aarch64 and a JetPack version
    # is real work for a network of 16 -> 256 -> 128 -> 64 -> 5: about 40k
    # parameters, four matrix multiplies, on a 0.4 s control step.  `numpy_policy`
    # in `deploy` runs this file with no torch at all.
    import numpy as np

    arrays: dict[str, "np.ndarray"] = {}
    norm = jit.obs_normalizer
    mean = getattr(norm, "_mean", None)
    std = getattr(norm, "_std", None)
    if mean is None or std is None:
        raise SystemExit(
            "정규화기에서 평균/표준편차를 찾지 못했습니다 — 이대로 내보내면 "
            "입력 스케일이 달라져 정책이 무의미해집니다"
        )
    arrays["norm_mean"] = mean.detach().cpu().numpy().reshape(-1)
    arrays["norm_std"] = std.detach().cpu().numpy().reshape(-1)
    arrays["norm_eps"] = np.asarray(float(getattr(norm, "eps", 1e-2)))
    layers = [m for m in jit.mlp.modules() if isinstance(m, torch.nn.Linear)]
    for k, lin in enumerate(layers):
        arrays[f"w{k}"] = lin.weight.detach().cpu().numpy()
        arrays[f"b{k}"] = lin.bias.detach().cpu().numpy()
    arrays["n_layers"] = np.asarray(len(layers))
    np.savez(out / "policy.npz", **arrays)

    # A policy that disagrees with its own weights is worse than no export, so
    # the two paths are compared here rather than trusted.
    from .deploy import numpy_policy

    rng = np.random.default_rng(0)
    probe_batch = rng.normal(size=(64, obs_dim)).astype("float32")
    with torch.no_grad():
        want = jit(torch.from_numpy(probe_batch)).numpy()
    got = numpy_policy(out / "policy.npz")(probe_batch)
    err = float(np.abs(want - got).max())
    if err > 1e-4:
        raise SystemExit(f"numpy 경로가 TorchScript 와 {err:.2e} 만큼 다릅니다")

    manifest = build_manifest(cfg, ckpt, obs_dim, action_dim)
    (out / "interface.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    print(f"[export] policy.pt      관측 {obs_dim} -> 행동 {action_dim}")
    print(f"[export] policy.npz     numpy 전용, TorchScript 와 최대 오차 {err:.1e}")
    print(f"[export] interface.json {len(manifest['observation']['channels'])} 채널, "
          f"들기 회전 {manifest['lift_script']['roll_seconds_for_90deg']:.2f} s / 90 deg")
    print(f"[export] 위치           {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
