"""Episode counters, training summaries and reproducibility metadata."""
from __future__ import annotations

import math
from typing import Any, TYPE_CHECKING

import torch

from ..configs.loader import to_dict

if TYPE_CHECKING:
    from .wrap_env import WrapGraspEnv

FAILURE_MODES = ("collision", "topple", "retention", "timeout")


class EpisodeMetrics:
    def __init__(self, env: WrapGraspEnv) -> None:
        self.env = env
        self._failure_counts = {
            mode: torch.zeros(1, device=env.device, dtype=torch.long)
            for mode in FAILURE_MODES
        }
        self._success_count = torch.zeros(1, device=env.device, dtype=torch.long)
        self._episode_count = torch.zeros(1, device=env.device, dtype=torch.long)
        #: Episodes since the rate was last read, for "best so far" tracking.
        #: `statistics()` is cumulative over the whole run, which cannot see a
        #: policy peak and then decline.
        self._recent_ok = 0
        self._recent_n = 0

    def logging_extras(
        self,
        reward_out: Any,
        contact: Any,
        state: dict[str, torch.Tensor],
        timeout: torch.Tensor,
    ) -> dict[str, torch.Tensor]:
        log: dict[str, torch.Tensor] = {}
        for name, value in reward_out.terms.items():
            log[f"reward/{name}"] = value.mean()
        for name, value in self.env.rewards.episode_sums().items():
            log[f"episode_sum/{name}"] = value.mean()
        log["wrap/phi_rad"] = state["phi"].mean()
        log["wrap/phi_max_rad"] = state["phi"].max()
        log["wrap/target_rad"] = torch.tensor(
            float(self.env.cfg.success.coverage_target_rad), device=self.env.device
        )
        log["wrap/surface_dist_m"] = state["surface_dist"].mean()
        log["wrap/min_surface_dist_m"] = state["min_surface_dist"].mean()
        log["wrap/enclosure_rad"] = state["enclosure_raw"].mean()
        log["wrap/plane_alignment"] = state["plane_alignment"].mean()
        log["curriculum/start_t_lo"] = torch.tensor(self.env._start_t_lo, device=self.env.device)
        log["curriculum/start_t_hi"] = torch.tensor(self.env._start_t_hi, device=self.env.device)
        log["wrap/enclosure_effective_rad"] = state["enclosure"].mean()
        # Waypoint usage per DoF.  The wrap needs roll near +/-90 deg to put the
        # bend plane horizontal, so a policy whose roll stays near its Home zero
        # cannot be wrapping whatever else the other terms say.
        wp = self.env.mapper.waypoint
        log["waypoint/linear_m"] = wp[:, 0].mean()
        log["waypoint/roll_rad"] = wp[:, 1].mean()
        log["waypoint/roll_abs_rad"] = wp[:, 1].abs().mean()
        log["waypoint/roll_abs_max_rad"] = wp[:, 1].abs().max()
        log["waypoint/theta1_rad"] = wp[:, 2].mean()
        log["waypoint/theta2_rad"] = wp[:, 3].mean()
        # Logged whether or not it gates success: a wrap that reaches the
        # coverage target while `caged` stays at zero is not holding anything.
        log["wrap/caged"] = state["caged"].to(torch.float32).mean()
        log["wrap/gap_rad"] = state["gap_rad"].mean()
        log["wrap/gap_width_m"] = state["gap_width_m"].mean()
        log["object/displacement_m"] = state["displacement"].mean()
        log["object/tilt_rad"] = state["tilt"].mean()
        log["contact/object_touch"] = contact.object_touch.to(torch.float32).mean()
        log["contact/non_target"] = contact.non_target_collision.to(torch.float32).mean()
        log["contact/floor"] = contact.floor_touch.to(torch.float32).mean()
        log["contact/support"] = contact.support_touch.to(torch.float32).mean()
        log["contact/go2"] = contact.go2_touch.to(torch.float32).mean()
        log["contact/self"] = contact.self_touch.to(torch.float32).mean()
        log["contact/self_structural"] = (
            contact.self_structural_touch.to(torch.float32).mean()
        )
        # A saturated contact buffer means readings may be incomplete; surface
        # it rather than trusting a silently truncated collision check.
        log["contact/buffer_overflow"] = contact.overflow.to(torch.float32).mean()
        log["term/collision"] = reward_out.termination_reason["collision"].to(torch.float32).mean()
        log["term/topple"] = reward_out.termination_reason["topple"].to(torch.float32).mean()
        log["term/success"] = reward_out.termination_reason["success"].to(torch.float32).mean()
        log["term/timeout"] = timeout.to(torch.float32).mean()
        return log

    def tally(self, reward_out: Any, timeout: torch.Tensor, dones: torch.Tensor) -> None:
        reasons = reward_out.termination_reason
        self._episode_count += int(dones.sum())
        self._success_count += int(reasons["success"].sum())
        self._recent_n += int(dones.sum())
        self._recent_ok += int((reasons["success"] & dones).sum())
        self._failure_counts["collision"] += int(reasons["collision"].sum())
        self._failure_counts["topple"] += int(reasons["topple"].sum())
        if self.env.script is not None:
            retention = self.env.script.finished & (~self.env.script.passed)
            self._failure_counts["retention"] += int((retention & dones).sum())
        self._failure_counts["timeout"] += int(
            (timeout & ~reward_out.terminate).sum()
        )

    def take_recent_success_rate(self, min_episodes: int = 1) -> tuple[float, int]:
        """Success rate over the episodes finished since the last reading.

        The counters are cleared only once `min_episodes` have accumulated, so
        a caller polling faster than episodes finish keeps building the sample
        instead of discarding it: at 128 envs an iteration finishes about 80
        episodes, and read-and-reset meant a 200-episode threshold was never
        reached at all.  Returns (rate, episodes); below the threshold the rate
        is 0.0 and the count says why.
        """
        n = self._recent_n
        if n < int(min_episodes):
            return 0.0, n
        rate = self._recent_ok / n
        self._recent_ok = 0
        self._recent_n = 0
        return rate, n

    def statistics(self) -> dict[str, float]:
        episodes = max(int(self._episode_count.item()), 1)
        stats = {
            "episodes": float(self._episode_count.item()),
            "success_rate": float(self._success_count.item()) / episodes,
        }
        for mode, count in self._failure_counts.items():
            stats[f"failure/{mode}"] = float(count.item()) / episodes
        return stats

    def metadata(self) -> dict[str, Any]:
        """Run-reproduction metadata, including the beta provenance flag."""
        return {
            "scene": self.env.scene.describe(),
            "beta": self.env.beta.describe(),
            "obs_dims": {"policy": self.env.obs_spec.policy, "privileged": self.env.obs_spec.privileged},
            "success_criterion": self.env.cfg.success.criterion,
            "coverage_target_deg": math.degrees(self.env.cfg.success.coverage_target_rad),
            "curriculum_stage": int(self.env.cfg.curriculum.stage),
            "config": to_dict(self.env.cfg),
        }
