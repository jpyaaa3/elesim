"""Reward terms for wrap grasping.

Each term is a separate function so its sign and magnitude can be unit-tested
in isolation, and :class:`RewardBook` keeps a per-episode sum of every term for
the logger.  Aggregate return alone cannot say whether a policy learned to
wrap or merely learned to stop moving; the per-term breakdown can.

Term definitions follow the task spec:

======================  ===========================================  ======
term                    definition                                   weight
======================  ===========================================  ======
coverage_progress       delta(wrap angle about the object's           +2.0
                        *current* centre) / 2*pi
approach_shaping        delta(-dist(segment-2 mid link, surface))     +0.5
                        / d0, and identically zero once that env
                        has made its first contact
step_cost               constant                                     -0.05
non_target_collision    arm vs floor / support / quadruped, plus the     -1.0
                        backbone folding into its own housing, judged
                        over the whole substep window -> terminate.
                        Node-against-node is not a collision: see
                        reward.self_contact
object_disturbance      pre-wrap object displacement, continuous      -0.5
object_topple           displacement or tilt over threshold ->       -2.0
                        terminate
success                 success.criterion met -> terminate            +5.0
                        (`geometric`: contact-anchored wrap angle at
                        or past success.coverage_target_rad)
======================  ===========================================  ======
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Mapping, Optional

import torch

from ..configs.loader import RewardConfig

_TWO_PI = 2.0 * math.pi

#: Term names, fixed so the logger's keys never drift from the weights.
TERM_NAMES: tuple[str, ...] = (
    "coverage_progress",
    "approach_shaping",
    "step_cost",
    "non_target_collision",
    "object_disturbance",
    "object_topple",
    "success",
)


def coverage_progress(phi: torch.Tensor, phi_prev: torch.Tensor) -> torch.Tensor:
    """Fraction of a full turn gained since the previous macro step.

    Signed on purpose: giving back coverage costs what gaining it paid, so the
    policy cannot farm reward by wrapping and unwrapping repeatedly.
    """
    return (phi - phi_prev) / _TWO_PI


def approach_shaping(
    dist: torch.Tensor,
    dist_prev: torch.Tensor,
    *,
    d0: float,
    touched: torch.Tensor,
) -> torch.Tensor:
    """Normalised progress towards the object surface, off after first contact.

    `touched` is sticky per env: once that env has made contact the term is
    zero for the rest of the episode, so shaping cannot compete with the wrap
    objective during the part of the task that actually matters.
    """
    delta = (dist_prev - dist) / max(float(d0), 1e-9)
    return torch.where(touched, torch.zeros_like(delta), delta)


def step_cost(n_envs: int, *, device: torch.device) -> torch.Tensor:
    """Constant per-macro-step cost (weight carries the sign)."""
    return torch.ones(n_envs, device=device, dtype=torch.float32)


def object_disturbance(
    displacement: torch.Tensor, *, deadband_m: float
) -> torch.Tensor:
    """Displacement beyond a dead-band, in metres.

    The dead-band keeps contact-induced jitter from being punished as if the
    policy had shoved the object.
    """
    return (displacement - float(deadband_m)).clamp_min(0.0)


@dataclass
class RewardInputs:
    """Everything a reward step needs, already aggregated over substeps."""

    phi: torch.Tensor                  # (n,) wrap angle now
    surface_dist: torch.Tensor         # (n,) segment-2 mid link to surface
    object_touch: torch.Tensor         # (n,) bool, contact during this step
    non_target_collision: torch.Tensor  # (n,) bool, over the substep window
    object_displacement: torch.Tensor  # (n,) metres since reset
    object_tilt: torch.Tensor          # (n,) radians from the reset axis
    success: torch.Tensor              # (n,) bool, lift/geometric gate passed


@dataclass
class RewardOutputs:
    total: torch.Tensor
    terms: dict[str, torch.Tensor]
    terminate: torch.Tensor
    termination_reason: dict[str, torch.Tensor]


@dataclass
class _EpisodeState:
    phi_prev: torch.Tensor
    dist_prev: torch.Tensor
    touched: torch.Tensor
    wrapped: torch.Tensor
    sums: dict[str, torch.Tensor] = field(default_factory=dict)


class RewardBook:
    """Evaluates the reward terms and accumulates their episode sums."""

    def __init__(
        self,
        cfg: RewardConfig,
        *,
        n_envs: int,
        device: torch.device,
        wrap_threshold_rad: float,
    ) -> None:
        self.cfg = cfg
        self.n_envs = int(n_envs)
        self.device = device
        self.wrap_threshold_rad = float(wrap_threshold_rad)
        self.weights = {name: float(getattr(cfg.weights, name)) for name in TERM_NAMES}
        self._state = _EpisodeState(
            phi_prev=torch.zeros(n_envs, device=device, dtype=torch.float32),
            dist_prev=torch.zeros(n_envs, device=device, dtype=torch.float32),
            touched=torch.zeros(n_envs, device=device, dtype=torch.bool),
            wrapped=torch.zeros(n_envs, device=device, dtype=torch.bool),
            sums={
                name: torch.zeros(n_envs, device=device, dtype=torch.float32)
                for name in TERM_NAMES
            },
        )

    # -- lifecycle ---------------------------------------------------------

    def reset(
        self,
        env_ids: Optional[torch.Tensor],
        *,
        phi0: torch.Tensor,
        dist0: torch.Tensor,
    ) -> None:
        st = self._state
        if env_ids is None:
            st.phi_prev[:] = phi0
            st.dist_prev[:] = dist0
            st.touched[:] = False
            st.wrapped[:] = False
            for name in TERM_NAMES:
                st.sums[name][:] = 0.0
            return
        if env_ids.numel() == 0:
            return
        st.phi_prev[env_ids] = phi0[env_ids] if phi0.numel() == self.n_envs else phi0
        st.dist_prev[env_ids] = dist0[env_ids] if dist0.numel() == self.n_envs else dist0
        st.touched[env_ids] = False
        st.wrapped[env_ids] = False
        for name in TERM_NAMES:
            st.sums[name][env_ids] = 0.0

    @property
    def touched(self) -> torch.Tensor:
        return self._state.touched

    def episode_sums(self) -> Mapping[str, torch.Tensor]:
        return self._state.sums

    # -- evaluation --------------------------------------------------------

    def step(self, inputs: RewardInputs) -> RewardOutputs:
        st = self._state
        cfg = self.cfg
        dist_cfg = cfg.disturbance

        raw: dict[str, torch.Tensor] = {}
        raw["coverage_progress"] = coverage_progress(inputs.phi, st.phi_prev)
        raw["approach_shaping"] = approach_shaping(
            inputs.surface_dist,
            st.dist_prev,
            d0=cfg.approach_d0,
            touched=st.touched,
        )
        raw["step_cost"] = step_cost(self.n_envs, device=self.device)
        raw["non_target_collision"] = inputs.non_target_collision.to(torch.float32)

        # Disturbance is only charged before the object is wrapped: once the arm
        # is around it, moving it is the point of the task.
        pre_wrap = ~st.wrapped
        disturb = object_disturbance(
            inputs.object_displacement, deadband_m=dist_cfg.deadband_m
        )
        raw["object_disturbance"] = torch.where(
            pre_wrap, disturb, torch.zeros_like(disturb)
        )

        toppled = (
            inputs.object_displacement > float(dist_cfg.max_displacement_m)
        ) | (inputs.object_tilt > float(dist_cfg.max_tilt_rad))
        toppled = toppled & pre_wrap
        raw["object_topple"] = toppled.to(torch.float32)
        raw["success"] = inputs.success.to(torch.float32)

        terms = {name: raw[name] * self.weights[name] for name in TERM_NAMES}
        total = torch.stack([terms[name] for name in TERM_NAMES], dim=0).sum(dim=0)

        for name in TERM_NAMES:
            st.sums[name] += terms[name]

        # State carried to the next macro step.
        st.phi_prev = inputs.phi.clone()
        st.dist_prev = inputs.surface_dist.clone()
        st.touched = st.touched | inputs.object_touch
        st.wrapped = st.wrapped | (inputs.phi >= self.wrap_threshold_rad)

        reasons = {
            "collision": inputs.non_target_collision.clone(),
            "topple": toppled,
            "success": inputs.success.clone(),
        }
        terminate = reasons["collision"] | reasons["topple"] | reasons["success"]
        return RewardOutputs(
            total=total, terms=terms, terminate=terminate, termination_reason=reasons
        )
