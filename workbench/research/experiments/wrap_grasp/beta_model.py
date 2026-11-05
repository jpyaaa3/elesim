"""Residual joint model (beta) and its estimator counterpart.

The real arm's gear backbone has backlash, and the segments deflect under
load.  Together those make the realised bend angle differ from the commanded
one by a residual the paper calls beta.  Hardware identification for beta is
not available yet, so this module is deliberately an *interface* with
placeholder parameters in the config:

* :meth:`BetaModel.corrupt` is the injection hook — commanded angle in,
  realised angle out.  Training rolls out against corrupted joints so the
  policy cannot rely on perfect actuation.
* :meth:`BetaModel.estimate` is the observation hook — it stands in for the
  on-robot beta compensator, so the actor observes an *estimate* of joint
  state rather than either the command or the simulator truth.

Both hooks are batched over envs and joints and run on whatever torch device
the caller uses.  `BetaConfig.measured` stays False until real data lands;
callers surface that flag in run metadata so no result is mistaken for one
calibrated against hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import torch

from .configs.loader import BetaConfig

_DEG2RAD = math.pi / 180.0


@dataclass
class BetaState:
    """Per-env residual parameters and the direction memory backlash needs."""

    beta0_rad: torch.Tensor      # (n_envs,)
    slope_rad_per_kg: torch.Tensor  # (n_envs,)
    last_direction: torch.Tensor    # (n_envs, n_joints), in {-1, 0, +1}


class BetaModel:
    """Batched backlash + deflection residual with a matching estimator."""

    def __init__(
        self,
        cfg: BetaConfig,
        *,
        n_envs: int,
        n_joints: int,
        device: torch.device,
        generator: Optional[torch.Generator] = None,
    ) -> None:
        self.cfg = cfg
        self.n_envs = int(n_envs)
        self.n_joints = int(n_joints)
        self.device = device
        self._generator = generator
        self.state = self._sample_state(torch.arange(self.n_envs, device=device))

    # -- internals ---------------------------------------------------------

    def _rand(self, *shape: int) -> torch.Tensor:
        return torch.rand(
            *shape, device=self.device, dtype=torch.float32, generator=self._generator
        )

    def _randn(self, *shape: int) -> torch.Tensor:
        return torch.randn(
            *shape, device=self.device, dtype=torch.float32, generator=self._generator
        )

    def _sample_state(self, env_ids: torch.Tensor) -> BetaState:
        n = int(env_ids.numel())
        jitter = float(self.cfg.beta0_jitter_deg) * _DEG2RAD
        base = float(self.cfg.beta0_deg) * _DEG2RAD
        beta0 = base + (self._rand(n) * 2.0 - 1.0) * jitter
        beta0 = beta0.clamp_min(0.0)
        slope = torch.full(
            (n,),
            float(self.cfg.load_slope_deg_per_kg) * _DEG2RAD,
            device=self.device,
            dtype=torch.float32,
        )
        return BetaState(
            beta0_rad=beta0,
            slope_rad_per_kg=slope,
            last_direction=torch.zeros(
                (n, self.n_joints), device=self.device, dtype=torch.float32
            ),
        )

    def reset(self, env_ids: Optional[torch.Tensor] = None) -> None:
        """Redraw beta for the given envs (all of them when `env_ids` is None)."""
        if env_ids is None:
            self.state = self._sample_state(torch.arange(self.n_envs, device=self.device))
            return
        if env_ids.numel() == 0:
            return
        fresh = self._sample_state(env_ids)
        self.state.beta0_rad[env_ids] = fresh.beta0_rad
        self.state.slope_rad_per_kg[env_ids] = fresh.slope_rad_per_kg
        self.state.last_direction[env_ids] = 0.0

    def magnitude(self, load_kg: torch.Tensor) -> torch.Tensor:
        """Residual magnitude per env, broadcast to (n_envs, 1).

        `load_kg` is the payload the arm is carrying, so the deflection term
        grows with the object mass the way the measured table does.
        """
        load = load_kg.reshape(-1).to(self.device, torch.float32)
        beta = self.state.beta0_rad + self.state.slope_rad_per_kg * load.clamp_min(0.0)
        return beta.clamp_min(0.0).unsqueeze(-1)

    # -- hooks -------------------------------------------------------------

    def corrupt(
        self,
        commanded: torch.Tensor,
        *,
        load_kg: torch.Tensor,
        previous_commanded: Optional[torch.Tensor] = None,
    ) -> torch.Tensor:
        """Commanded joint angles -> angles the joints actually reach.

        Backlash lags motion, so the residual is subtracted along the current
        direction of travel.  When the command does not move a joint the
        previous direction is held, which is what makes backlash hysteretic
        rather than a plain offset.
        """
        if not self.cfg.enable:
            return commanded
        beta = self.magnitude(load_kg)
        if not self.cfg.directional:
            return commanded - beta
        if previous_commanded is None:
            direction = self.state.last_direction
        else:
            delta = commanded - previous_commanded
            moved = delta.abs() > 0.0
            direction = torch.where(
                moved, torch.sign(delta), self.state.last_direction
            )
        self.state.last_direction = direction
        return commanded - direction * beta

    def estimate(
        self,
        realised: torch.Tensor,
        *,
        load_kg: torch.Tensor,
        noise: bool = True,
    ) -> torch.Tensor:
        """Realised joint angles -> what the on-robot estimator would report.

        `estimator_gain` is how much of the residual the compensator removes:
        1.0 recovers the command exactly, 0.0 passes the corrupted angle
        through untouched.  The actor sees only this, never simulator truth.
        """
        if not self.cfg.enable:
            return realised
        beta = self.magnitude(load_kg)
        corrected = realised + float(self.cfg.estimator_gain) * self.state.last_direction * beta
        if noise and self.cfg.estimator_noise_deg > 0.0:
            sigma = float(self.cfg.estimator_noise_deg) * _DEG2RAD
            corrected = corrected + self._randn(*corrected.shape) * sigma
        return corrected

    def describe(self) -> dict[str, object]:
        """Run-metadata summary; `measured` is what reviewers need to see."""
        return {
            "enable": bool(self.cfg.enable),
            "measured": bool(self.cfg.measured),
            "beta0_deg": float(self.cfg.beta0_deg),
            "load_slope_deg_per_kg": float(self.cfg.load_slope_deg_per_kg),
            "beta0_jitter_deg": float(self.cfg.beta0_jitter_deg),
            "directional": bool(self.cfg.directional),
            "estimator_gain": float(self.cfg.estimator_gain),
            "estimator_noise_deg": float(self.cfg.estimator_noise_deg),
            "apply_bundle_sag_model": bool(self.cfg.apply_bundle_sag_model),
            "source": "measured" if self.cfg.measured else "placeholder (config)",
        }
